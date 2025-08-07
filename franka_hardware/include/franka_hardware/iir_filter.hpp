#pragma once

#include "online_filter.hpp"

namespace acg_signal_processing {

template <std::size_t P, std::size_t Q>
class IIRFilter : public OnlineFilter<P+1, 1, 1> {
public:
    static constexpr std::size_t NUM_SIZE = P + 1;
    static constexpr std::size_t DEN_SIZE = Q;

    IIRFilter(std::array<double, NUM_SIZE> coefficients_num, std::array<double, DEN_SIZE> coefficients_den)
        : OnlineFilter<NUM_SIZE, 1, 1>(), coefficients_num_(coefficients_num), coefficients_den_(coefficients_den) {}

    ~IIRFilter() {}

    DataPoint<1> sample(const Timestamp&) override {
        // Get the last timestamp from the input buffer
        Timestamp last_timestamp = input_buffer_.get_last().timestamp;
        
        // Call the base class sample method to get the output
        DataPoint<1> result = OnlineFilter<NUM_SIZE, 1, 1>::sample(last_timestamp);

        // Save the output to the output buffer
        if(DEN_SIZE > 0){
            StampedDataPoint<1> output_sample;
            output_sample.timestamp = last_timestamp;
            output_sample.data[0] = result[0];
            output_buffer_.add(output_sample);
        }

        return result;
    }

    bool is_ready() const override {
        return OnlineFilter<NUM_SIZE, 1, 1>::is_ready() && output_buffer_.size() >= DEN_SIZE;
    }

    void set_filter_state(double state) {
        std::array<StampedDataPoint<1>, DEN_SIZE> initial_state;
        initial_state.fill({0.0, {state}});
        output_buffer_.set_full(initial_state);
    }

protected:
    RingBuffer<StampedDataPoint<1>, DEN_SIZE> output_buffer_;
    RingBuffer<StampedDataPoint<1>, NUM_SIZE>& input_buffer_ = this->buffer_;
    const std::array<double, NUM_SIZE> coefficients_num_;
    const std::array<double, DEN_SIZE> coefficients_den_;

    void sample_impl(const Timestamp&, DataPoint<1>& output) override {
        if(!is_ready()) {
            throw std::runtime_error("Not enough samples to compute output");
        }
        double result = 0.0;
        for (std::size_t i = 0; i < NUM_SIZE; ++i) {
            std::size_t input_index = (NUM_SIZE - 1 - i) % NUM_SIZE;
            result += coefficients_num_[i] * input_buffer_.get(input_index).data[0];
        }
        if(DEN_SIZE > 0) {
            for (std::size_t i = 0; i < DEN_SIZE; ++i) {
                std::size_t output_index = (DEN_SIZE - 1 - i) % DEN_SIZE;
                result -= coefficients_den_[i] * output_buffer_.get(output_index).data[0];
            }
        }
        output[0] = result;
    }
};

} // namespace acg_signal_processing