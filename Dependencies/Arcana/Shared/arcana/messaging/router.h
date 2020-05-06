#pragma once

#include "arcana/finally_scope.h"
#include "arcana/functional/inplace_function.h"
#include "arcana/sentry.h"
#include "arcana/threading/affinity.h"

#include <algorithm>
#include <tuple>
#include <vector>

#include <gsl/gsl>

namespace mira
{
    using ticket_seed = int64_t;

    using ticket = gsl::final_action<
        stdext::inplace_function<void(), sizeof(std::aligned_storage_t<sizeof(ticket_seed) + sizeof(void*)>)>>;

    using ticket_scope = finally_scope<ticket>;

    /*
        An event routing class used to dispatch events to multiple listeners.
        Each router class can only handle a certain fixed set of event types defined by EventTs.
    */
    template<typename... EventTs>
    class router
    {
    public:
        static constexpr size_t LISTENER_SIZE = 4 * sizeof(int64_t);

        template<typename EventT>
        using listener_function = stdext::inplace_function<void(const EventT&), LISTENER_SIZE>;


        /*
            Sends an event synchronously to all listeners.
        */
        template<typename EventT>
        void fire(const EventT& evt)
        {
            GSL_CONTRACT_CHECK("thread affinity", m_affinity.check());

            using event = std::decay_t<EventT>;

            auto& listeners = std::get<listener_group<event>>(m_listeners);

            {
                auto guard = std::get<sentry<event>>(m_sentries).take();

                for (listener<event>& listener : listeners)
                {
                    if (listener.valid)
                    {
                        listener.callback(evt);
                    }
                }
            }

            // if we're no longer iterating the listeners list in this stack
            // remove all the unregistered listeners and add the pending ones
            if (!std::get<sentry<event>>(m_sentries).is_active())
            {
                listeners.erase(std::remove_if(listeners.begin(),
                                               listeners.end(),
                                               [](const listener<event>& l) { return !l.valid; }),
                                listeners.end());

                // move the pending listeners to the real list
                // and clear the pending list
                auto& pending = std::get<listener_group<event>>(m_pending);
                std::move(pending.begin(), pending.end(), std::back_inserter(listeners));
                pending.clear();
            }
        }

        /*
            Adds an event listener.
        */
        template<typename EventT, typename T>
        ticket add_listener(T&& listener)
        {
            auto id = internal_add_listener<EventT>(std::forward<T>(listener));

            return ticket{ [id, this] { internal_remove_listener<EventT>(id); } };
        }

        /*
            Sets the routers thread affinity. Once this is set the methods
            on this instance will need to be called by that thread.
        */
        void set_affinity(const affinity& aff)
        {
            m_affinity = aff;
        }

    private:
        /*
        Adds an event listener.
        */
        template<typename EventT, typename T>
        ticket_seed internal_add_listener(T&& listener)
        {
            GSL_CONTRACT_CHECK("thread affinity", m_affinity.check());

            using event = std::decay_t<EventT>;

            // if we're currently firing an event in that group we need to wait until we're done before
            // adding the listener to the list
            auto& listeners = std::get<sentry<event>>(m_sentries).is_active()
                                  ? std::get<listener_group<event>>(m_pending)
                                  : std::get<listener_group<event>>(m_listeners);

            auto id = m_nextId++;
            listeners.emplace_back(std::forward<T>(listener), id);
            return id;
        }

        /*
        Removes an event listener by id.
        */
        template<typename EventT>
        void internal_remove_listener(const ticket_seed& id)
        {
            GSL_CONTRACT_CHECK("thread affinity", m_affinity.check());

            using event = std::decay_t<EventT>;

            auto& listeners = std::get<listener_group<event>>(m_listeners);
            auto found = std::find_if(listeners.begin(), listeners.end(), [id](const listener<event>& listener) {
                return listener.id == id;
            });

            if (found == listeners.end())
            {
                assert(false && "removing item that isn't there");
                return;
            }

            // don't modify the collection while iterating, just disable the listener
            if (std::get<sentry<event>>(m_sentries).is_active())
            {
                found->valid = false;
            }
            else
            {
                listeners.erase(found);
            }
        }

        template<typename EventT>
        struct listener
        {
            using callback_t = listener_function<EventT>;

            callback_t callback;
            ticket_seed id;
            bool valid;

            listener(callback_t&& callback, const ticket_seed& id)
                : callback{ std::move(callback) }
                , id{ id }
                , valid{ true }
            {}

            listener(const callback_t& callback, const ticket_seed& id)
                : callback{ callback }
                , id{ id }
                , valid{ true }
            {}
        };

        template<typename EventT>
        using listener_group = std::vector<listener<EventT>>;

        std::tuple<listener_group<EventTs>...> m_listeners;
        std::tuple<sentry<EventTs>...> m_sentries;
        std::tuple<listener_group<EventTs>...> m_pending;

        affinity m_affinity;
        ticket_seed m_nextId;
    };
}
