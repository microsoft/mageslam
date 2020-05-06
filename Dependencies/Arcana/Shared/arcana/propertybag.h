// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "iterators.h"
#include "macros.h"
#include "type_traits.h"
#include "string.h"
#include "path.h"

#include <cereal/cereal.hpp>

#include <algorithm>
#include <assert.h>
#include <cmath>
#include <gsl/gsl>
#include <iterator>
#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace mira
{
    struct property_bag;

    struct property_interface
    {
        property_interface(const char* type, const char* name)
            : m_type{ type }
            , m_name{ name }
        {}

        virtual ~property_interface()
        {}

        const char* name() const
        {
            return m_name;
        }

        const char* type() const
        {
            return m_type;
        }

    private:
        const char* const m_type;
        const char* const m_name;
    };

    struct simple_property : public property_interface
    {
        virtual void from_string(const std::string& str) = 0;
        virtual void from_stream(std::istream& str) = 0;

        virtual void from_other(const property_interface& other) = 0;

        virtual std::type_index serializable_type() const = 0;

        virtual std::string to_string() const = 0;

        simple_property(const char* type, const char* name)
            : property_interface(type, name)
        {}
    };

    template<typename SerializableType>
    struct serializable_property : public simple_property
    {
        serializable_property(const char* type, const char* name)
            : simple_property(type, name)
        {}

        virtual ~serializable_property() {}

        std::type_index serializable_type() const override
        {
            return { typeid(SerializableType) };
        }

        // serialization functions
        template<class Archive>
        SerializableType save_minimal(Archive const&) const
        {
            return get_serializable();
        }

        template<class Archive>
        void load_minimal(Archive const&, SerializableType const& val)
        {
            set_serializable(val);
        }

    protected:
        virtual SerializableType get_serializable() const = 0;
        virtual void set_serializable(const SerializableType& value) = 0;
    };

    template<typename T> struct property;

    struct property_bag : public property_interface
    {
        bool set(const std::string& name, const std::string& value)
        {
            auto itr = m_properties.find(name.c_str());
            if (itr == m_properties.end())
                return false;

            itr->second->from_string(value);

            return true;
        }

        void from_other(const property_bag& other)
        {
            assert(strcmp(name(), other.name()) == 0);
            assert(other.m_properties.size() == m_properties.size());
            assert(other.m_groups.size() == m_groups.size());

            for (auto prop : m_properties)
            {
                prop.second->from_other(*other.m_properties.at(prop.first));
            }

            for (size_t i = 0; i < m_groups.size(); ++i)
            {
                assert(strcmp(m_groups[i]->name(), other.m_groups[i]->name()) == 0);
                m_groups[i]->from_other(*other.m_groups[i]);
            }
        }

        std::string get(const std::string& name) const
        {
            return m_properties.find(name.c_str())->second->to_string();
        }

        std::vector<simple_property*> properties()
        {
            std::vector<simple_property*> props;
            std::transform(m_properties.begin(), m_properties.end(), std::back_inserter(props), [](const auto& entry) {
                return entry.second;
            });
            return props;
        }

        std::vector<const simple_property*> properties() const
        {
            std::vector<const simple_property*> props;
            std::transform(m_properties.begin(), m_properties.end(), std::back_inserter(props), [](const auto& entry) {
                return entry.second;
            });
            return props;
        }

        gsl::span<property_bag*> groups()
        {
            return m_groups;
        }

        gsl::span<const property_bag*> groups() const
        {
            return { const_cast<property_bag const**>(m_groups.data()), (gsl::span<const property_bag*>::size_type)m_groups.size() };
        }

        // serialization function
        template<typename ArchiveT>
        void serialize(ArchiveT& ar)
        {
            using arch_func = void(ArchiveT & ar, property_interface * prop);

            static std::unordered_map<std::type_index, arch_func*> converters{
                { typeid(bool), &archive_prop<ArchiveT, serializable_property<bool>> },
                { typeid(int), &archive_prop<ArchiveT, serializable_property<int>> },
                { typeid(unsigned int), &archive_prop<ArchiveT, serializable_property<unsigned int>> },
                { typeid(uint32_t), &archive_prop<ArchiveT, serializable_property<uint32_t>> },
                { typeid(uint64_t), &archive_prop<ArchiveT, serializable_property<uint64_t>> },
                { typeid(size_t), &archive_prop<ArchiveT, serializable_property<size_t>> },
                { typeid(float), &archive_prop<ArchiveT, serializable_property<float>> },
                { typeid(std::string), &archive_prop<ArchiveT, serializable_property<std::string>> },
                { typeid(double), &archive_prop<ArchiveT, serializable_property<double>> },
            };

            for (auto prop : m_properties)
            {
                auto found = converters.find(prop.second->serializable_type());
                if (found == converters.end())
                    throw std::invalid_argument("unable to convert type, conversion function not found");

                found->second(ar, prop.second);
            }

            for (auto group : m_groups)
            {
                ar(cereal::make_nvp(group->name(), *group));
            }
        }

    protected:
        property_bag(const char* name)
            : property_bag{ name, nullptr }
        {}

        property_bag(const char* name, property_bag* owner)
            : property_interface{ "bag", name }
        {
            if (owner != nullptr)
            {
                owner->add(this);
            }
        }

        property_bag(const property_bag& other) = delete;
        property_bag& operator=(const property_bag& other) = delete;

    private:
        template<typename ArchiveT, typename T>
        static void archive_prop(ArchiveT& ar, property_interface* prop)
        {
            ar(cereal::make_nvp(prop->name(), *static_cast<T*>(prop)));
        }

        template<typename T>
        friend struct property;

        void add(simple_property* prop)
        {
            m_properties.insert({ prop->name(), prop });
        }

        void add(property_bag* prop)
        {
            m_groups.push_back(prop);
        }

        std::map<const char*, simple_property*, string_compare> m_properties;

        std::vector<property_bag*> m_groups;
    };

    template<typename T, bool isEnum = std::is_enum<T>::value>
    struct property_serialization_traits;

    template<typename T>
    struct property_serialization_traits<T, true>
    {
        using serialization_type = std::underlying_type_t<T>;

        static serialization_type to_serializable(const T& value)
        {
            return static_cast<serialization_type>(value);
        }

        static T from_serializable(const serialization_type& value)
        {
            return static_cast<T>(value);
        }

        static void from_stream(std::istream& stream, T& value)
        {
            serialization_type underlyingValue{};

            stream >> underlyingValue;

            assert(!stream.fail() && "Property bag failed to convert string to value");

            value = static_cast<T>(underlyingValue);
        }

        static void to_stream(std::ostream& stream, const T& value)
        {
            auto underlying = static_cast<serialization_type>(value);

            std::stringstream ss;
            stream.precision(std::numeric_limits<serialization_type>::max_digits10);
            stream << underlying;

            assert(!stream.fail() && "Property bag failed to convert value to string");
        }
    };

    template<typename T>
    struct property_serialization_traits<T, false>
    {
        using serialization_type = T;

        static serialization_type to_serializable(const T& value)
        {
            return value;
        }

        static T from_serializable(const serialization_type& value)
        {
            return value;
        }

        static void from_stream(std::istream& stream, T& value)
        {
            if (std::is_same<T, bool>())
            {
                stream >> std::boolalpha >> value;
            }
            else
            {
                stream >> value;
            }

            assert((is_string<T>() || !stream.fail()) && "Property bag failed to convert string to value");
        }

        static void to_stream(std::ostream& stream, const T& value)
        {
            if (std::is_same<T, bool>())
            {
                stream << std::boolalpha << value;
            }
            else
            {
                stream.precision(std::numeric_limits<T>::max_digits10);
                stream << value;
            }
            assert(!stream.fail() && "Property bag failed to convert value to string");
        }
    };

    template<>
    struct property_serialization_traits<path, false>
    {
        using serialization_type = std::string;

        static serialization_type to_serializable(const path& value)
        {
            return value.string();
        }

        static path from_serializable(const serialization_type& value)
        {
            return { value };
        }

        static void from_stream(std::istream& stream, path& value)
        {
            stream >> value;
        }

        static void to_stream(std::ostream& stream, const path& value)
        {
            stream << value;
        }
    };

    template<typename T>
    struct property : public serializable_property<typename property_serialization_traits<T>::serialization_type>
    {
        using traits = property_serialization_traits<T>;

        using type = std::decay_t<T>;

        property(property_bag* lookup, const char* name, const char* type, const T& defaultValue)
            : serializable_property<typename traits::serialization_type>{ type, name }
            , value{ defaultValue }
        {
            lookup->add(this);
        }

        property& operator=(const property& other)
        {
            value = other.value;
            return *this;
        }

        virtual void from_string(const std::string& str) override
        {
            std::stringstream ss{ str };
            from_stream(ss);
        }

        virtual void from_stream(std::istream& stream) override
        {
            traits::from_stream(stream, value);
        }

        virtual void from_other(const property_interface& other) override
        {
            assert(dynamic_cast<const property<T>*>(&other) && "invalid type");
            value = static_cast<const property<T>&>(other).value;
        }

        virtual std::string to_string() const override
        {
            std::stringstream ss;
            traits::to_stream(ss, value);
            return ss.str();
        }

        operator T() const
        {
            return value;
        }

        property& operator=(const T& val)
        {
            value = val;
            return *this;
        }

        T value;

    protected:
        typename traits::serialization_type get_serializable() const override
        {
            return traits::to_serializable(value);
        }

        void set_serializable(const typename traits::serialization_type& val) override
        {
            value = traits::from_serializable(val);
        }
    };

    template<typename T>
    std::ostream& operator<<(std::ostream& os, const property<T>& prop)
    {
        return os << prop.value;
    }
}

#define PROPERTY(TYPE, NAME, DEFAULT) ::mira::property<TYPE> NAME{ this, #NAME, #TYPE, DEFAULT };

#define WEIRDLY_NAMED_PROPERTY(TYPE, STRNAME, DEFAULT)                                                                 \
    ::mira::property<TYPE> CONCATENATE_MACRO(setting_, __COUNTER__){ this, STRNAME, #TYPE, DEFAULT };

#define BAG_PROPERTY(TYPE) TYPE TYPE{ this };

#define NAMED_BAG_PROPERTY(TYPE, NAME) TYPE NAME{ this, #NAME };

#define PROPERTYBAG(NAME, ...)                                                                                         \
    struct NAME : public ::mira::property_bag                                                                          \
    {                                                                                                                  \
        NAME(const NAME& other)                                                                                        \
            : NAME{ other.name() }                                                                                     \
        {                                                                                                              \
            from_other(other);                                                                                         \
        }                                                                                                              \
        NAME& operator=(const NAME& other)                                                                             \
        {                                                                                                              \
            from_other(other);                                                                                         \
            return *this;                                                                                              \
        }                                                                                                              \
        NAME()                                                                                                         \
            : ::mira::property_bag{ #NAME }                                                                            \
        {}                                                                                                             \
        NAME(::mira::property_bag* owner)                                                                              \
            : ::mira::property_bag{ #NAME, owner }                                                                     \
        {}                                                                                                             \
        NAME(const char* name)                                                                                         \
            : ::mira::property_bag{ name }                                                                             \
        {}                                                                                                             \
        NAME(::mira::property_bag* owner, const char* name)                                                            \
            : ::mira::property_bag{ name, owner }                                                                      \
        {}                                                                                                             \
        __VA_ARGS__                                                                                                    \
    }
