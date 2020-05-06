// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include <functional>
#include <tuple>

#include "arcana\utils\stl_shim.h"
#include "Analysis\binary_iterators.h"

namespace mage
{
    class DataFlow
    {
    public:
        DataFlow() = default;

        DataFlow(const DataFlow&) = delete;
        DataFlow(DataFlow&&) = delete;

        DataFlow& operator=(const DataFlow&) = delete;
        DataFlow& operator=(DataFlow&&) = delete;

        ~DataFlow();

        template<typename ...ArgTs>
        DataFlow& Input(ArgTs&& ...args)
        {
            mira::make_binary_iterator(iterator{ *this, true }).iterate(std::forward<ArgTs>(args)...);

            return *this;
        }

        template<typename FuncT>
        DataFlow& Output(FuncT argFactory)
        {
            m_output.push_back([this, factory = std::move(argFactory)]
            {
                std::apply([this](auto&& ...args)
                {
                    mira::make_binary_iterator(iterator{ *this, false }).iterate(std::forward<decltype(args)>(args)...);
                }, factory());
            });

            return *this;
        }

        struct iterator
        {
            iterator(DataFlow& flow, bool input)
                : m_flow{ &flow }, m_input{ input }
            {}

            void operator()(const void* data, size_t bytes)
            {
                if (m_input)
                    m_flow->LogInput(data, bytes);
                else
                    m_flow->LogOutput(data, bytes);
            }

            DataFlow* m_flow;
            bool m_input;
        };
    private:
        friend class mira::binary_iterator<iterator>;

        void LogInput(const void* data, size_t size);
        void LogOutput(const void* data, size_t size);

        int64_t m_inputBytes = 0;
        int64_t m_outputBytes = 0;

        std::vector<std::function<void()>> m_output;
    };

    namespace proxy
    {
        inline void binary_iterate(const UnAssociatedMask& data, mira::binary_iterator<DataFlow::iterator>& itr)
        {
            itr.iterate(&data, sizeof(data));

            // std vector of bool is implemented as one bit per bool, so process it as such
            itr.iterate(nullptr, (size_t)ceil(data.GetUnassociatedKeypointMask().size() / 8.0f));
        }

        inline void binary_iterate(const Image& image, mira::binary_iterator<DataFlow::iterator>& itr)
        {
            // proxy::Image is just a pointer to an image
            itr.iterate(&image, sizeof(image));
        }
    }
}

namespace cv
{
    inline void binary_iterate(const cv::Mat& data, mira::binary_iterator<mage::DataFlow::iterator>& itr)
    {
        itr.iterate(nullptr, data.total() * data.elemSize());
    }
}

#ifdef INSTRUMENT_DATA_FLOW

#define DATAFLOW(...) ::mage::DataFlow internal_function_dataflow{}; internal_function_dataflow __VA_ARGS__
#define DF_INPUT(...) .Input(__VA_ARGS__)
#define DF_OUTPUT(...) .Output([&] { return std::make_tuple(__VA_ARGS__); })

#else

#define DATAFLOW(...) 

#endif