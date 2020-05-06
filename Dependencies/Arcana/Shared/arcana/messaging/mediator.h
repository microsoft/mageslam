// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include <tuple>

#include "arcana/finally_scope.h"
#include "arcana/functional/inplace_function.h"
#include "arcana/threading/dispatcher.h"

#include "router.h"

#include <gsl/gsl>

namespace mira
{
    template<typename DispatcherT, typename... EventTs>
    class mediator
    {
        using router_t = router<EventTs...>;

    public:
        using dispatcher_t = DispatcherT;

        explicit mediator(dispatcher_t& dispatcher)
            : m_dispatcher{ dispatcher }
        {}

        template<typename T>
        void send(T&& evt)
        {
            m_dispatcher.queue([ this, evt = std::forward<T>(evt) ]() { m_router.fire(evt); });
        }

        template<typename EventT, typename T>
        ticket add_listener(T&& listener)
        {
            GSL_CONTRACT_CHECK("thread affinity", m_dispatcher.get_affinity().check());
            return m_router.template add_listener<EventT>(std::forward<T>(listener));
        }

        dispatcher_t& dispatcher()
        {
            return m_dispatcher;
        }

    private:
        dispatcher_t& m_dispatcher;
        router_t m_router;
    };
}
