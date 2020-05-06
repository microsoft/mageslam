// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "arcana/type_traits.h"
#include "arcana/utils/serialization/base_stream.h"
#include "arcana/utils/serialization/serializable.h"

#include <gsl/gsl>
#include <memory>
#include <type_traits>

namespace mira
{
    template<typename LambdaT>
    class binary_iterator
    {
        template<typename T>
        using is_serializable = std::is_base_of<serializable<std::remove_cv_t<T>>, T>;

        template<typename L>
        struct check_custom_iterator
        {
            template<typename T,
                     typename = decltype(binary_iterate(instance_of<const T>(),
                                                        instance_of<binary_iterator<LambdaT>>()))>
            static std::true_type test(T* = nullptr);
            static std::false_type test(...);
        };

        template<typename T>
        using has_custom_iterator = decltype(check_custom_iterator<LambdaT>::test(&instance_of<T>()));

        template<typename T>
        using is_valid_pod =
            std::integral_constant<bool,
                                   !has_custom_iterator<T>::value && std::is_pod<std::decay_t<T>>::value &&
                                       !std::is_pointer<std::decay_t<T>>::value>;

    public:
        template<typename... ArgTs>
        void iterate_all(ArgTs&&... args)
        {
            int unused[] = { (iterate(std::forward<ArgTs>(args)), 0)... };
            (void)unused;
        }

        template<typename T>
        std::enable_if_t<has_custom_iterator<std::decay_t<T>>::value> iterate(const T& value)
        {
            static_assert(!is_valid_pod<T>::value, "we've got some overlap");
            binary_iterate(value, *this);
        }

        template<typename T>
        std::enable_if_t<is_valid_pod<std::decay_t<T>>::value> iterate(const T& ptr)
        {
            static_assert(!has_custom_iterator<T>::value, "we've got some overlap");
            stream{ *this }.write(ptr);
        }

        void iterate(const void* data, size_t size)
        {
            m_iter(data, size);
        }

        template<typename ItrT>
        void iterate(ItrT beg, ItrT end)
        {
            while (beg != end)
            {
                iterate(*beg);
                beg++;
            }
        }

        template<typename A, typename B>
        void iterate(const std::pair<A, B>& pair)
        {
            iterate(pair.first);
            iterate(pair.second);
        }

        template<typename T>
        void iterate(const std::shared_ptr<T>& ptr)
        {
            iterate(ptr.get());
        }

        template<typename T, ptrdiff_t E = -1>
        void iterate(const gsl::span<T, E>& span)
        {
            iterate(span.begin(), span.end());
        }

        template<typename T, typename D>
        void iterate(const std::unique_ptr<T, D>& ptr)
        {
            iterate(ptr.get());
        }

        template<typename T>
        void iterate(const T* ptr)
        {
            if (!ptr)
                iterate((intptr_t)0);
            else
                iterate(*ptr);
        }

        template<typename T>
        void iterate(const serializable<T>& ptr)
        {
            stream s{ *this };
            ptr.serialize(s);
        }

        template<typename ItrT>
        explicit binary_iterator(ItrT&& i)
            : m_iter{ std::forward<ItrT>(i) }
        {}

        LambdaT& iterator()
        {
            return m_iter;
        }

        const LambdaT& iterator() const
        {
            return m_iter;
        }

    private:
        struct stream : public base_stream
        {
            template<typename T>
            void write(const T& obj)
            {
                m_itr.iterate(&obj, sizeof(T));
            }

            template<typename T>
            void write(const T* obj, size_t N)
            {
                m_itr.iterate(obj, N * sizeof(T));
            }

            stream(binary_iterator<LambdaT>& itr)
                : m_itr{ itr }
            {}

        private:
            binary_iterator<LambdaT>& m_itr;
        };

        LambdaT m_iter;
    };

    template<typename LambdaT>
    binary_iterator<std::decay_t<LambdaT>> make_binary_iterator(LambdaT&& lambda)
    {
        return binary_iterator<std::decay_t<LambdaT>>{ std::forward<LambdaT>(lambda) };
    }
}

namespace std
{
    template<typename Iter>
    void binary_iterate(const std::string& str, mira::binary_iterator<Iter>& iter)
    {
        iter.iterate(str.data(), str.size());
    }

    template<typename T, typename Iter>
    void binary_iterate(const std::vector<T>& vec, mira::binary_iterator<Iter>& iter)
    {
        iter.iterate(vec.size());
        iter.iterate(vec.begin(), vec.end());
    }

    template<typename IterT, typename FirstT, typename SecondT>
    void binary_iterate(const std::pair<FirstT, SecondT>& data, mira::binary_iterator<IterT>& itr)
    {
        itr.iterate_all(data.first, data.second);
    }

    template<typename Iter, typename T>
    void binary_iterate(const std::shared_ptr<T>& ptr, mira::binary_iterator<Iter>& iter)
    {
        iter.iterate(ptr.get());
    }

    template<typename Iter, typename T>
    void binary_iterate(const std::unique_ptr<T>& ptr, mira::binary_iterator<Iter>& iter)
    {
        iter.iterate(ptr.get());
    }
}

namespace gsl
{
    template<typename Iter, typename T, ptrdiff_t E = -1>
    void binary_iterate(const gsl::span<T, E>& span, mira::binary_iterator<Iter>& iter)
    {
        iter.iterate(span.begin(), span.end());
    }
}
