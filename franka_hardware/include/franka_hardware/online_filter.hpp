#pragma once

#include <array>
#include <exception>

namespace acg_signal_processing {

using Timestamp = double;

using Duration = double;

template <std::size_t N>
using DataPoint = std::array<double, N>;

template <std::size_t N>
struct StampedDataPoint {
    Timestamp timestamp; // Timestamp
    DataPoint<N> data; // Data point
};

template <typename T, std::size_t N>
class RingBuffer {
public:
    RingBuffer() : head_(0), count_(0) { }

    void add(const T value) {
        // Insert the value at the current tail position
        buffer_[tail()] = value;

        // If the buffer is not full, increment the count
        if (count_ < N)
        {
            count_++;
        }
        // If the buffer is full, overwrite the oldest value
        else
        {
            head_ = (head_ + 1) % N;
        }

    }

    void set_full(const std::array<T, N>& values) {
        if (values.size() != N) {
            throw std::runtime_error("Invalid number of values provided");
        }
        buffer_ = values;
        count_ = N; // Set count to N since we have filled the buffer
        head_ = 0; // Reset head to the start of the buffer
    }

    const T& get(std::size_t index) const {
        if (index >= count_) {
            throw std::out_of_range("Index out of range");
        }
        return buffer_[(head_ + index) % N];
    }

    const T& get_first() const {
        if (is_empty()) {
            throw std::runtime_error("Buffer is empty");
        }
        return buffer_[head_];
    }

    const T& get_last() const {
        if (is_empty()) {
            throw std::runtime_error("Buffer is empty");
        }
        size_t last_index = (head_ + count_ - 1) % N;
        return buffer_[last_index];
    }

    void clear() {
        head_ = 0;
        count_ = 0;
    }

    std::size_t size() const {
        return count_;
    }

    const T& operator[](std::size_t index) const {
        return get(index);
    }

    T& operator[](std::size_t index) {
        return const_cast<T&>(static_cast<const RingBuffer&>(*this)[index]);
    }

    bool is_empty() const {
        return count_ == 0;
    }

    bool is_full() const {
        return count_ == N;
    }

private:
    std::array<T, N> buffer_;
    std::size_t head_;
    std::size_t count_;

    std::size_t tail() const {
        return (head_ + count_) % N;
    }
};


template <std::size_t W, std::size_t Din, std::size_t Dout>
class OnlineFilter {
public:
    OnlineFilter() {}

    ~OnlineFilter() {}

    virtual void add_sample(const StampedDataPoint<Din>& sample){
        if (!buffer_.is_empty() && sample.timestamp <= buffer_.get_last().timestamp) {
            throw std::runtime_error("New sample timestamp must be greater than the last sample timestamp");
        }
        buffer_.add(sample);
    }

    virtual void set_samples(const std::array<StampedDataPoint<Din>, W>& samples) {
        if (samples.size() != W) {
            throw std::runtime_error("Invalid number of samples provided");
        }
        buffer_.set_full(samples);
    }

    virtual DataPoint<Dout> sample(const Timestamp& timestamp) {
        if (!buffer_.is_full()) {
            throw std::runtime_error("Not enough samples to compute output");
        }

        const StampedDataPoint<Din>& first_sample = buffer_.get_first();
        const StampedDataPoint<Din>& last_sample = buffer_.get_last();

        if(first_sample.timestamp > timestamp || last_sample.timestamp < timestamp) {
            throw std::runtime_error("Timestamp out of range");
        }

        // Calculate the relative timestamp from the first sample
        Timestamp relative_timestamp = (timestamp - first_sample.timestamp) / get_buffer_duration();

        // Call the implementation to compute the output based on the samples
        DataPoint<Dout> output;
        sample_impl(relative_timestamp, output);
        return output;
    }

    virtual bool is_ready() const {
        return buffer_.size() >= W;
    }

    virtual bool is_empty() const {
        return buffer_.is_empty();
    }

protected:
    virtual void sample_impl(const Timestamp& normalized_timestamp, DataPoint<Dout>& output) = 0;

    RingBuffer<StampedDataPoint<Din>, W> buffer_;

    Duration get_buffer_duration() const {
        if (buffer_.size() < 2) {
            return 0.0; // Not enough samples to compute duration
        }
        return buffer_.get_last().timestamp - buffer_.get_first().timestamp;
    }
};
}  // namespace acg_signal_processing