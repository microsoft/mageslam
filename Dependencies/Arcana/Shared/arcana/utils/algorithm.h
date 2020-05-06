// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include <set>
#include <typeinfo>
#include <vector>
#include <assert.h>

#include <numeric>
#include <cmath>

namespace mira
{
    /*
        the std::max from algorithm isn't a constexpr yet
    */
    template<typename T>
    constexpr const T& max(const T& left, const T& right)
    {
        return left > right ? left : right;
    }

    template<class InputIt1, class InputIt2>
    unsigned int set_intersection_count(InputIt1 first1, InputIt1 last1, InputIt2 first2, InputIt2 last2)
    {
        unsigned int count = 0;
        while (first1 != last1 && first2 != last2)
        {
            if (*first1 < *first2)
            {
                ++first1;
            }
            else
            {
                if (!(*first2 < *first1))
                {
                    count++;
                    *first1++;
                }
                ++first2;
            }
        }
        return count;
    }

    template<typename InputIt1, typename InputIt2, typename CompareT>
    unsigned int set_intersection_count(
        InputIt1 first1, InputIt1 last1, InputIt2 first2, InputIt2 last2, CompareT compare)
    {
        unsigned int count = 0;
        while (first1 != last1 && first2 != last2)
        {
            if (compare(*first1, *first2))
            {
                ++first1;
            }
            else
            {
                if (!compare(*first2, *first1))
                {
                    count++;
                    *first1++;
                }
                ++first2;
            }
        }
        return count;
    }

    template<typename T, float T::*Member>
    struct greater_member
    {
        bool operator()(const T& left, const T& right)
        {
            return left.*Member > right.*Member;
        }
    };

    template<typename T, float T::*Member>
    struct lesser_member
    {
        bool operator()(const T& left, const T& right)
        {
            return left.*Member < right.*Member;
        }
    };

    template<typename T>
    struct identity
    {
        T operator()(T v)
        {
            return v;
        }
    };

    template<typename... Args>
    struct iterate_typenames
    {
        template<typename FuncT>
        void operator()(FuncT&& func)
        {
            int itms[] = { (func(typeid(Args).name()), 0)... };
            (void)itms;
        }
    };

    template<>
    struct iterate_typenames<>
    {
        template<typename FuncT>
        void operator()(FuncT&&)
        {}
    };

    template<template<typename> class Trait, typename... Args>
    struct iterate_traits
    {
        template<typename FuncT>
        void operator()(FuncT&& func)
        {
            int itms[] = { (func(Trait<Args>::value), 0)... };
            (void)itms;
        }
    };

    template<template<typename> class Trait>
    struct iterate_traits<Trait>
    {
        template<typename FuncT>
        void operator()(FuncT&&)
        {}
    };

    /*
        templated class for turning a number N into a series [0, N)
    */
    template<std::size_t... Indices>
    struct indices
    {
    };

    template<std::size_t N, std::size_t... Is>
    struct build_indices : build_indices<N - 1, N - 1, Is...>
    {
    };

    template<std::size_t... Is>
    struct build_indices<0, Is...> : indices<Is...>
    {
    };

    /**
     * Returns a vector with the computed subsets of size K.
     *
     * @param begin     An iterator to the beginning of your set This can be any container.
     * @param end       An iterator to the end of your set This can be any container.
     * @param K         The number of elements in the subset. This can be in [0, set size].
     * @return A vector with the computed subsets of size K.
     *
     * REMARKS:
     * This code uses permutations to do a combination with no repetition of
     * the K elements from the input set.
     *
     * For instance, for set {a, b, c} it will generate the following subsets
     * of size 2:
     *      a) {a b}
     *      b) {b c}
     *      c) {a c}
     */
    template<class T, class IteratorT>
    std::vector<std::set<T>> compute_subsets(const IteratorT& begin, const IteratorT& end, size_t K)
    {
        if (begin == end)
        {
            return {};
        }

        /**
         * Set up a mask with  K requested elements.
         * For instance, for a set of length 3 and K = 2
         * mask = 110
         */
        size_t setLength = std::distance(begin, end); // N items in the set
        std::vector<bool> bitmask(K, 1);              // K leading 1's
        bitmask.resize(setLength, 0);                 // N-K trailing 0'sB

        std::vector<std::set<T>> vectorSet;

        do
        {
            std::set<T> combSet;

            /**
             * Here we let std calculate the permutation of the mask (110 in the example)
             * and when we find a bit turned on, we copy that as a valid element from the set
             * for the subset being computed. This is in fact, the combination with no repetition.
             */
            for (size_t i = 0; i < setLength; ++i) // [0..N-1] integers
            {
                if (bitmask[i])
                {
                    combSet.insert(*std::next(begin, i));
                }
            }

            vectorSet.push_back(std::move(combSet));
        } while (std::prev_permutation(bitmask.begin(), bitmask.end()));

        return vectorSet;
    }

    // Note: This function will reorder the container according to nth_element.
    template<typename T, typename IteratorT, typename ModifierT>
    T median(const IteratorT& begin, const IteratorT& end, ModifierT&& modifier)
    {
        size_t length = std::distance(begin, end);

        if (length == 0)
        {
            assert(false && "no median for an empty data set");
            return {};
        }

        if (length % 2 == 1)
        {
            auto idx = length / 2;
            auto nth = begin + idx;
            std::nth_element(begin, nth, end);
            return modifier(*nth);
        }
        else
        {
            auto nth = begin + length / 2;
            std::nth_element(begin, nth, end);
            auto nth_minus_one = std::max_element(begin, nth);
            return (modifier(*nth_minus_one) + modifier(*nth)) / 2;
        }
    }

    template<class T, class IteratorT>
    T median(const IteratorT& begin, const IteratorT& end)
    {
        return median<T>(begin, end, identity<typename IteratorT::value_type>{});
    }

    template<typename T, typename IteratorT, typename ModifierT>
    inline const T sum(IteratorT from, IteratorT to, ModifierT&& modifier)
    {
        return std::accumulate(from, to, T{}, [&](const T& accum, const typename IteratorT::value_type& value)
        {
            return accum + modifier(value);
        });
    }

    template<typename T, typename IteratorT>
    inline const T sum(IteratorT from, IteratorT to)
    {
        return sum<T>(from, to, identity<typename IteratorT::value_type>{});
    }

    template<typename T, typename IteratorT, typename ModifierT>
    inline const T mean(IteratorT from, IteratorT to, ModifierT&& modifier)
    {
        const auto n = std::distance(from, to);
        const auto sumValue = sum<T>(from, to, modifier);

        if (n == 0)
        {
            assert(false && "no median for an empty data set");
            return {};
        }

        return sumValue / n;
    }

    template<typename T, typename IteratorT>
    inline const T mean(IteratorT from, IteratorT to)
    {
        return mean<T>(from, to, identity<typename IteratorT::value_type>{});
    }

    // Function to caluclate the standard deviation using the sum and squared sum
    // https://en.wikipedia.org/wiki/Standard_deviation#Rapid_calculation_methods
    template<typename T>
    inline const T population_standard_deviation(T sum, T squaredSum, size_t count)
    {
        return std::sqrt(count * squaredSum - sum * sum) / count;
    }

    template<typename T, typename IteratorT, typename ModifierT>
    inline const T standard_deviation(IteratorT from, IteratorT to, ModifierT&& modifier)
    {
        const auto n = std::distance(from, to);
        if (n <= 1)
        {
            assert(false && "your collection cannot have less than two items");
            return {};
        }

        const auto calculatedMean = mean<T>(from, to, modifier);
        const auto accum = sum<T>(from, to, [&](const typename IteratorT::value_type& currentValue)
                            {
                                T delta = modifier(currentValue) - calculatedMean;
                                return delta * delta;
                            });

        return std::sqrt(accum / (n - 1));
    }

    template<typename T, typename IteratorT>
    inline const T standard_deviation(IteratorT from, IteratorT to)
    {
        return standard_deviation<T>(from, to, identity<typename IteratorT::value_type>{});
    }
}
