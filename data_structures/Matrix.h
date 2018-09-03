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
/**
 * @brief IndexHelper struct template helps to defer matrix element retrieval, given that only a row index was provided. When constructed, it behaves like an array, that can be accessed by providing index.
 * @tparam Next Callable, that accepts a single argument used to finalize access to matrix element
 */
template <typename Next>
struct IndexHelper {
    explicit IndexHelper(Next next) : next_(std::move(next)) {}
    std::result_of_t<Next(size_t)> operator[](size_t x) { return next_(x); }

  private:
    Next next_;
};

/**
 * @brief indexHelper function constructs IndexHelper struct instance with provided template argument instance
 * @tparam Next Type of callable
 */
template <typename Next>
IndexHelper<Next> indexHelper(Next &&next) {
    return IndexHelper<Next>(std::forward<Next>(next));
}

/**
 * @brief The Matrix class template can represent matrices of different types and sizes
 * @tparam T Type of elements stored inside matrix
 */
template <typename T>
struct Matrix {
    /** Alias for type of elements stored in matrix */
    using value_type = T;
    /** Alias for reference to type of elements stored in matrix */
    using reference = T &;
    /** Alias for constant reference to type of elements stored inside matrix */
    using const_reference = const T &;
    /** Alias for type used to store matrix size */
    using size_type = std::size_t;
    /** Alias for type used to represent difference of two iterators */
    using difference_type = typename std::vector<T>::difference_type;
    /** Alias for iterator, that passes elements row by row */
    using iterator = typename std::vector<T>::iterator;
    /** Alias for constant iterator, that passes elements row by row */
    using const_iterator = typename std::vector<T>::const_iterator;
    /** Alias for reverse iterator, that passes elements in reverse order */
    using reverse_iterator = std::reverse_iterator<iterator>;
    /** Alias for constant reverse iterator, that passes elements in reverse order */
    using const_reverse_iterator = std::reverse_iterator<const_iterator>;

    /**
     * @brief Default constructor */
    Matrix() = default;
    /**
     * @brief Constructs Matrix instance from columns and rows sizes
     * @param x Number of columns to initialize
     * @param y Number of rows to initialize
     */
    Matrix(size_t x, size_t y) : storage_(x * y), cols_(x) {}
    /**
     * @brief Constructs Matrix instance from columns and rows sizes
     * @param x Number of columns to initialize
     * @param y Number of rows to initialize
     */
    Matrix(size_t x, size_t y, const T &value)
        : storage_(x * y, value), cols_(x) {}
    /**
     * @brief Returns size of matrix as a pair of columns and rows
     * @return std::pair with the first size_t number denoting number of columns and the second number of rows
     */
    std::pair<size_t, size_t> size() const {
        return std::make_pair(cols_,
                              storage_.size() / (cols_ ? cols_ : cols_ + 1));
    }
    /**
     * @brief Returns true if the Matrix instance has elements with default constructed values
     * @return bool value, that signifies non-default values in matrix
     */
    bool empty() const {
        T t_compare = T();
        return std::all_of(storage_.begin(), storage_.end(),
                           [&t_compare](const T &t) { return t == t_compare; });
    }
    /**
     * @brief Returns reference to element at column x and row y
     * @param x Column index
     * @param y Row index
     * @return reference to the element at position x and y
     */
    reference access(size_t x, size_t y) { return storage_[y * cols_ + x]; }
    /**
     * @brief Returns constant reference to element at column x and row y
     * @param x Column index
     * @param y Row index
     * @return constant reference to the element at position x and y
     */
    const_reference access(size_t x, size_t y) const {
        return storage_[y * cols_ + x];
    }
    /**
     * @brief Returns IndexHelper instance, that accepts row index and returns corresponding element
     * @param x Column index
     * @return IndexHelper struct instance, that awaits row index through operator[]
     */
    auto operator[](size_t x) {
        return indexHelper([=](size_t y) -> reference { return access(x, y); });
    }
    /**
     * @brief Returns IndexHelper instance, that accepts row index and returns corresponding constant reference to the correponding element
     * @param x Column index
     * @return IndexHelper struct instance, that awaits row index through operator[]
     */
    auto operator[](size_t x) const {
        return indexHelper(
            [=](size_t y) -> const_reference { return access(x, y); });
    }
    /**
     * @brief Returns iterator to the top leftmost element
     * @return iterator to the top leftmost element
     */
    iterator begin() { return storage_.begin(); }
    /**
     * @brief Returns constant iterator to the top leftmost element
     * @return const_iterator to the top leftmost element
     */
    const_iterator begin() const { return storage_.begin(); }
    /**
     * @brief Returns constant iterator to the top leftmost element
     * @return const_iterator to the top leftmost element
     */
    const_iterator cbegin() const { return storage_.cbegin(); }
    /**
     * @brief Returns reverse iterator, that points to the lowest rightmost element
     * @return reverse_iterator to the lowest rightmost element
     */
    reverse_iterator rbegin() { return storage_.rbegin(); }
    /**
     * @brief Returns constant reverse iterator, that points to the lowest rightmost element
     * @return const_reverse_iterator to the lowest rightmost element
     */
    const_reverse_iterator crbegin() const { return storage_.crbegin(); }

    /**
     * @brief Returns iterator, that points past the lowest rightmost element
     * @return iterator, that points past the lowest rightmost element
     */
    iterator end() { return storage_.end(); }
    /**
     * @brief Returns constant iterator, that points past the lowest rightmost element
     * @return const_iterator, that points past the lowest rightmost element
     */
    const_iterator end() const { return storage_.end(); }
    /**
     * @brief Returns constant iterator, that points past the lowest rightmost element
     * @return const_iterator, that points past the lowest rightmost element
     */
    const_iterator cend() const { return storage_.cend(); }
    /**
     * @brief Returns reverse iterator, that points to the element before the top leftmost one
     * @return reverse_iterator, that points to the element before the top leftmost one
     */
    reverse_iterator rend() { return storage_.rend(); }
    /**
     * @brief Returns constant reverse iterator, that points to the element before the top leftmost one
     * @return const_reverse_iterator, that points to the element before the top leftmost one
     */
    const_reverse_iterator crend() const { return storage_.crend(); }

    /**
     * @brief Compares this to provided Matrix instance for equality
     * @param m Constant reference to Matrix instance
     * @return bool value, that signifies equality
     */
    bool operator==(const Matrix &m) const {
        return cols_ == m.cols_ && storage_ == m.storage_;
    }
    /**
     * @brief Compares this to provided Matrix instance for inequality
     * @param m Constant reference to Matrix instance
     * return bool value, that signifies non-equality
     */
    bool operator!=(const Matrix &m) const { return !(operator==(m)); }

  private:
    std::vector<T> storage_;
    size_t cols_{0};
};

} // namespace graphlib
#endif
