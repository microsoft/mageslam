// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "arcana/type_traits.h"
#include "arcana/either.h"
#include "arcana/iterators.h"

#include <memory>
#include <gsl/gsl>
#include <utility>

#include <cereal/cereal.hpp>
#include <cereal/archives/json.hpp>


namespace mira
{
    //
    // Generic object used to iterate an object hierarchy in order to
    // iterate all its members in the goal of introspecting objects as they
    // flow through a system.
    //
    // CallableT is anything that can get invoked to introspect an object hierarchy.
    // It supports cereal serialization out of the box in order to help writing out
    // object hierarchies to file, so one thing to be aware of is that when iterating
    // if you need to handle T's and cereal::NameValuePair<T>'s.
    //
    // The introspection contract isn't as strict as the serialization contract.
    // If a type is already serializable using cereal, it should already support
    // being introspected.
    //
    // If the type can't be serialized for any reason it can still be introspected.
    // All a type needs in order to be introspected is a free function with the following
    // signature:
    //
    // template<typename ArchiveT>
    // void introspect_object(introspector<ArchiveT>& archive, const Foo& object)
    // {
    //      archive(
    //              cereal::make_nvp("Param1", object.param1),
    //              object.param2 // if the name doesn't matter, but it usually helps.
    //      );
    // }
    //

    namespace internal
    {
        // helper struct to support nested introspectable types
        // that create objects in the serializer.
        struct nested_serialize_object
        {
            std::function<void()> callback;

            template<typename ArchiveT>
            void save(ArchiveT&) const
            {
                callback();
            }
        };

        // helper struct to support nested introspectable objects
        // that don't create objects but act as additional data to the parent.
        struct nested_serialize_inline
        {
            std::function<void()> callback;

            template<typename ArchiveT>
            void save(ArchiveT&) const
            {
                callback();
            }
        };

        inline void prologue(cereal::JSONOutputArchive&, const nested_serialize_inline&)
        {}

        inline void epilogue(cereal::JSONOutputArchive&, const nested_serialize_inline&)
        {}
    }

    template<typename CallableT>
    class introspector
    {
        template<typename L>
        struct has_internal_introspection_function_test
        {
            template<typename T,
                typename = decltype(introspect_object_internal(instance_of<introspector<CallableT>>(), instance_of<T>()))>
                static std::true_type test(T* = nullptr);

            static std::false_type test(...);
        };

        template<typename T>
        using has_internal_introspection_function = decltype(has_internal_introspection_function_test<CallableT>::test(&instance_of<T>()));

        template<typename L>
        struct has_custom_introspection_function_test
        {
            template<typename T,
                typename = decltype(introspect_object(instance_of<introspector<CallableT>>(), instance_of<T>()))>
            static std::true_type test(T* = nullptr);

            static std::false_type test(...);
        };

        template<typename T>
        using has_custom_introspection_function = decltype(has_custom_introspection_function_test<CallableT>::test(&instance_of<T>()));
    public:
        template<typename ItrT>
        explicit introspector(ItrT&& i)
            : m_callable{ std::forward<ItrT>(i) }
        {}

        using internally_introspected = std::integral_constant<size_t, 0>;
        using externally_introspected = std::integral_constant<size_t, 1>;
        using default_introspected = std::integral_constant<size_t, 2>;

        template<typename T>
        using introspection_type =
            find_first_index<has_internal_introspection_function<T>, has_custom_introspection_function<T>>;

        template<typename T>
        static constexpr bool should_introspect()
        {
            return introspection_type<T>::value < default_introspected::value;
        }

        template<typename ...ArgTs>
        void operator()(ArgTs&& ...args)
        {
            mira::static_foreach([this](auto&& arg)
            {
                introspect(std::forward<decltype(arg)>(arg));
            }, std::forward<ArgTs>(args)...);
        }

        template<typename T>
        void introspect(const T& value)
        {
            introspect_dispatch(value, introspection_type<T>{});
        }

        CallableT& callable()
        {
            return m_callable;
        }

        const CallableT& callable() const
        {
            return m_callable;
        }

    private:
        template<typename T>
        void introspect_dispatch(const T& value, internally_introspected)
        {
            introspect_object_internal(*this, value);
        }

        template<typename T>
        void introspect_dispatch(const T& value, externally_introspected)
        {
            internal::nested_serialize_object nested{ [&]
            {
                introspect_object(*this, value);
            } };
            m_callable(nested);
        }

        template<typename T>
        void introspect_dispatch(const T& value, default_introspected)
        {
            m_callable(value);
        }

        CallableT m_callable;
    };

    template<typename CallableT, typename T>
    void introspect_object_internal(introspector<CallableT>& ar, const cereal::NameValuePair<T>& named)
    {
        internal::nested_serialize_inline nested{ [&]
        {
            ar.introspect(named.value);
        } };
        ar.callable()(cereal::make_nvp(named.name, nested));
    }

    template<typename CallableT, typename T>
    void introspect_object_internal(introspector<CallableT>& ar, const T* object)
    {
        if (!object)
        {
            ar.callable()(cereal::make_nvp("pointer", nullptr));
        }
        else
        {
            internal::nested_serialize_inline nested{ [&]
            {
                ar.introspect(*object);
            } };
            ar.callable()(cereal::make_nvp("pointer", nested));
        }
    }

    template<typename CallableT, typename T>
    void introspect_object(introspector<CallableT>& ar, const std::shared_ptr<T>& ptr)
    {
        ar.introspect(ptr.get());
    }

    template<typename CallableT, typename T, typename D>
    void introspect_object(introspector<CallableT>& ar, const std::unique_ptr<T, D>& ptr)
    {
        ar.introspect(ptr.get());
    }

    template<typename CallableT, typename T, std::ptrdiff_t Extent>
    void introspect_object(introspector<CallableT>& ar, gsl::span<T, Extent> elements)
    {
        ar.callable()(cereal::SizeTag<cereal::size_type>(elements.size()));

        for (auto& el : elements)
        {
            ar.introspect(el);
        }
    }

    template<typename CallableT, typename T, typename AllocT>
    void introspect_object(introspector<CallableT>& ar, const std::vector<T, AllocT>& elements)
    {
        introspect_object(ar, gsl::make_span(elements));
    }

    template<typename CallableT, typename A, typename B>
    void introspect_object(introspector<CallableT>& ar, const std::pair<A, B>& elements)
    {
        ar(
            cereal::make_nvp("First", elements.first),
            cereal::make_nvp("Second", elements.second)
        );
    }
}
