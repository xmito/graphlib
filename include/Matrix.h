/* Implementation of Matrix, that is used to hold results of
 * graph algorithms like Johnson's or Floyd Warshall algorithm */
#ifndef MATRIX_H
#define MATRIX_H
#include <vector>
#include <type_traits>
#include <utility>
#include <numeric>
#include <algorithm>

namespace graphlib {

// this snippet of code is taken from PV264 seminar
// authors: Vladimir Still, Nikola Benes, Jan Mrazek
template <typename Next>
struct IndexHelper {
    explicit IndexHelper(Next next) : next_(std::move(next)) {}
    std::result_of_t<Next(size_t)> operator[](size_t x) { return next_(x); }

  private:
    Next next_;
};

template <typename Next>
IndexHelper<Next> indexHelper(Next &&next) {
    return IndexHelper<Next>(std::forward<Next>(next));
}

template <typename T>
struct Matrix {
    using value_type = T;
    using reference = T &;
    using const_reference = const T &;
    using size_type = std::size_t;
    using difference_type = typename std::vector<T>::difference_type;
    using iterator = typename std::vector<T>::iterator;
    using const_iterator = typename std::vector<T>::const_iterator;
    using reverse_iterator = std::reverse_iterator<iterator>;
    using const_reverse_iterator = std::reverse_iterator<const_iterator>;

    Matrix() = default;
    Matrix(size_t x, size_t y) : storage_(x * y), cols_(x) {}
    Matrix(size_t x, size_t y, const T &value)
        : storage_(x * y, value), cols_(x) {}
    std::pair<size_t, size_t> size() const {
        return std::make_pair(cols_,
                              storage_.size() / (cols_ ? cols_ : cols_ + 1));
    }
    bool empty() const {
        T t_compare = T();
        return std::all_of(storage_.begin(), storage_.end(),
                           [&t_compare](const T &t) { return t == t_compare; });
    }
    reference access(size_t x, size_t y) { return storage_[y * cols_ + x]; }
    const_reference access(size_t x, size_t y) const {
        return storage_[y * cols_ + x];
    }
    auto operator[](size_t x) {
        return indexHelper([=](size_t y) -> reference { return access(x, y); });
    }
    auto operator[](size_t x) const {
        return indexHelper(
            [=](size_t y) -> const_reference { return access(x, y); });
    }
    iterator begin() { return storage_.begin(); }
    const_iterator begin() const { return storage_.begin(); }
    const_iterator cbegin() const { return storage_.cbegin(); }
    reverse_iterator rbegin() { return storage_.rbegin(); }
    const_reverse_iterator crbegin() const { return storage_.crbegin(); }

    iterator end() { return storage_.end(); }
    const_iterator end() const { return storage_.end(); }
    const_iterator cend() const { return storage_.cend(); }
    reverse_iterator rend() { return storage_.rend(); }
    const_reverse_iterator crend() const { return storage_.crend(); }

    bool operator==(const Matrix &m) const {
        return cols_ == m.cols_ && storage_ == m.storage_;
    }
    bool operator!=(const Matrix &m) const { return !(operator==(m)); }

  private:
    std::vector<T> storage_;
    size_t cols_{0};
};

} // namespace graphlib
#endif
