// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include <arcana/type_traits.h>
#include <arcana/utils/algorithm.h>
#include <arcana/analysis/introspector.h>

#include "ProxyFactory.h"
#include "GenericFields.h"
#include "Utils/collection.h"


namespace mage
{
    template<typename T, typename ...Properties>
    struct View;

    template<typename T, typename ...Properties>
    struct MutableView;

    /*
            This class represents a copy of an object in the Map.
        It can represent a KeyFrame or a MapPoint, and defines what
        subset of properties it contains. Its goal is to minimize the
        amount of data we need to shuttle through our thread
        boundaries.
    */
    template<typename T, typename ...Properties>
    class Proxy : public proxy::Id<T>, public Properties...
    {
    public:
        using OriginalT = T;
        using ViewT = View<T, Properties...>;
        using MutableViewT = MutableView<T, Properties...>;

        Proxy(const T& original)
            : Id{ original }, Properties{ original }...
        {}

        Proxy(const T* original)
            : Id{ *original }, Properties{ *original }...
        {}

        template<typename StreamT, typename = mira::is_stream_t<StreamT>>
        Proxy(StreamT& stream)
            : Id{ stream }, Properties{ stream }...
        {}

        /*
            Copy constructor to build a new proxy object with a subset of
            properties. If you only need to pass an object to a function
            that accepts a subset, you should prefer the Proxy::ViewT which
            doesn't copy the object but just represents a subset
            of read only properties (but they have to be in the same order).
        */
        template<typename ...Others>
        explicit Proxy(const Proxy<T, Others...>& proxy)
            : Id{ proxy.GetId() }, Properties{ proxy }...
        {}
        
        /*
            This factory method should be used when you want to create
            an object that doesn't come from the map. This method allocates
            a new id and builds a project object with the goal to create
            it in the map.
        */
        template<typename ...CreationProperties>
        static Proxy CreateNew(CreationProperties&&... args)
        {
            return ProxyFactory<T>::CreateNew<Proxy>(std::forward<CreationProperties>(args)...);
        }

        template<typename ProxyT>
        const ProxyT& As() const
        {
            return *ProxyT::ViewT{ *this };
        }

        template<typename One, typename... Args>
        Proxy<T, Properties..., One> Extend(Args&&... args) const
        {
            return Proxy<T, Properties..., One>{ this->GetId(), *static_cast<const Properties*>(this)..., One{ std::forward<Args>(args)... } };
        }

        Proxy(const Proxy& other) = default;
        Proxy& operator =(const Proxy& other) = default;
        Proxy(Proxy&& other) = default;
        Proxy& operator =(Proxy&& other) = default;

        template<typename ...Args>
        explicit Proxy(const mage::Id<T>& id, Args&&... args)
            : Id{ id }, Properties{ std::forward<Args>(args) }...
        {}

    protected:
        friend class ProxyFactory<T>;

        template<typename T, typename ...Properties>
        friend class Proxy;
    };

    template<typename ArchiveT, typename T, typename ...ProxyTs>
    void introspect_object(mira::introspector<ArchiveT>& intro, const Proxy<T, ProxyTs...>& proxy)
    {
        intro(static_cast<const proxy::Id<T>&>(proxy));
        intro(static_cast<const ProxyTs&>(proxy)...);
    }

    template<typename T>
    struct ProxyTraits : std::false_type
    {
        using target = void;
    };

    template<typename T, typename ...Properties>
    struct ProxyTraits<Proxy<T, Properties...>> : std::true_type
    {
        using target = T;

        template<typename O, typename ...Subset>
        static constexpr bool is_castable_to()
        {
            return std::is_same<T, O>::value &&
                mira::starts_with<std::tuple<Subset...>, std::tuple<Properties...>>::value;
        }
    };
    
    template<typename T, typename ...Properties>
    struct collection_traits<Proxy<T, Properties...>>
    {
        template<typename InputT, typename = std::enable_if_t<ProxyTraits<std::remove_const_t<InputT>>::value>>
        static constexpr
        auto convert(InputT* input)
        {
            static_assert(ProxyTraits<std::remove_const_t<InputT>>::is_castable_to<T, Properties...>(), "Can't convert proxy types");
            return reinterpret_cast<Proxy<T, Properties...>*>(input);
        }
    };

    template<typename T, typename ...Properties>
    struct collection_traits<const Proxy<T, Properties...>>
    {
        template<typename InputT, typename = std::enable_if_t<ProxyTraits<std::remove_const_t<InputT>>::value>>
        static constexpr
            auto convert(InputT* input)
        {
            static_assert(ProxyTraits<std::remove_const_t<InputT>>::is_castable_to<T, Properties...>(), "Can't convert proxy types");
            return reinterpret_cast<const Proxy<T, Properties...>*>(input);
        }
    };

    /*
        Represents a view on an existing Proxy object. It supports
        subsets of Proxy objects in order to reduce the strength of function
        contracts. The Proxy object represented by this view needs to out-live
        this view object.
    */
    template<typename T, typename ...Properties>
    struct View
    {
    public:
        template<typename ...Remainder>
        View(const Proxy<T, Properties..., Remainder...>& proxy)
            : m_proxy{ reinterpret_cast<const Proxy<T, Properties...>&>(proxy) }
        {}

        const Proxy<T, Properties...>* operator ->() const
        {
            return &m_proxy;
        }

        const Proxy<T, Properties...>& operator *() const
        {
            return m_proxy;
        }

    private:
        const Proxy<T, Properties...>& m_proxy;
    };

    template<typename T, typename ...Properties>
    struct MutableView
    {
    public:
        template<typename ...Remainder>
        MutableView(Proxy<T, Properties..., Remainder...>& proxy)
            : m_proxy{ reinterpret_cast<Proxy<T, Properties...>&>(proxy) }
        {}

        Proxy<T, Properties...>* operator ->()
        {
            return &m_proxy;
        }

        Proxy<T, Properties...>& operator *()
        {
            return m_proxy;
        }

    private:
        Proxy<T, Properties...>& m_proxy;
    };
}
