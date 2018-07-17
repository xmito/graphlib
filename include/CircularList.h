#ifndef CIRCULAR_LIST_H
#define CIRCULAR_LIST_H

#include <algorithm>
#include <functional>
#include <list>
#include <type_traits>

template <typename T>
class CircularList
{
    using list_iterator = typename std::list<T>::iterator;

    enum class Behaviour : char
    {
        CLIST,
        LIST
    };
    template <bool IsConst>
    struct CListIterator
    {
        using value_type = T;
        using reference = std::conditional_t<IsConst, const T &, T &>;
        using pointer = std::conditional_t<IsConst, const T *, T *>;
        using difference_type = std::ptrdiff_t;
        using iterator_category = std::bidirectional_iterator_tag;

      private:
        using list_type = std::conditional_t<IsConst, const std::list<value_type>,
                                             std::list<value_type>>;
        using list_iterator =
            std::conditional_t<IsConst,
                               typename std::list<value_type>::const_iterator,
                               typename std::list<value_type>::iterator>;
        list_type *list_;
        list_iterator cit_;
        Behaviour bhv_;

        template <bool>
        friend struct CListIterator;
        template <typename>
        friend class CircularList;

      public:
		CListIterator(Behaviour bhv = Behaviour::CLIST)
            : list_(nullptr), bhv_(bhv) {}
		CListIterator(list_type &list, Behaviour bhv = Behaviour::CLIST)
            : list_(&list), cit_(list->begin()), bhv_(bhv) {}
        CListIterator(list_type &list, list_iterator cit,
                      Behaviour bhv = Behaviour::CLIST)
            : list_(&list), cit_(cit), bhv_(bhv) {}
		CListIterator(const CListIterator<false> &it)
            : list_(it.list_), cit_(it.cit_), bhv_(it.bhv_) {}
		CListIterator(std::nullptr_t) : list_(nullptr) {}
        CListIterator &operator=(std::nullptr_t)
        {
            cit_ = list_->end();
            list_ = nullptr;
            return *this;
        }
        CListIterator &operator++()
        {
            ++cit_;
            if (list_ && !list_->empty() && cit_ == list_->end() &&
                bhv_ == Behaviour::CLIST)
                cit_ = list_->begin();
            return *this;
        }
        CListIterator operator++(int)
        {
            auto clit(*this);
            ++(*this);
            return clit;
        }
        CListIterator &operator--()
        {
            --cit_;
            if (list_ && !list_->empty() && cit_ == --list_->begin() &&
                bhv_ == Behaviour::CLIST)
                cit_ = --list_->end();
            return *this;
        }
        CListIterator operator--(int)
        {
            auto clit(*this);
            --(*this);
            return clit;
        }
        reference operator*() { return *cit_; }
        pointer operator->() { return &(*cit_); }
        template <bool K>
        bool operator==(CListIterator<K> it) const
        {
            return cit_ == it.cit_;
        }
        template <bool K>
        bool operator!=(CListIterator<K> it) const
        {
            return !(*this == it);
        }
        bool operator==(std::nullptr_t) const { return list_ == nullptr; }
        bool operator!=(std::nullptr_t) const { return list_ != nullptr; }
		explicit operator bool() const { return list_ != nullptr; }
        void setBehaviour(Behaviour bhv) { bhv_ = bhv; }
        Behaviour getBehaviour() const { return bhv_; }
    };

  public:
    using value_type = T;
    using reference = T &;
    using const_reference = const T &;
    using iterator = CListIterator<false>;
    using const_iterator = CListIterator<true>;
    using size_type = size_t;

    // Constructors
    CircularList() = default;
	explicit CircularList(size_type count, const_reference value = T())
        : list_(count, value) {}
    template <typename Iterator>
    CircularList(Iterator first, Iterator last) : list_(first, last) {}

    // Lookup methods
    bool empty() const { return list_.empty(); }
    size_type size() const { return list_.size(); }
    reference front() { return list_.front(); }
    const_reference front() const { return list_.front(); }
    reference back() { return list_.back(); }
    const_reference back() const { return list_.back(); }
    iterator begin() { return iterator(list_, list_.begin(), Behaviour::LIST); }
    const_iterator begin() const
    {
        return const_iterator(list_, list_.begin(), Behaviour::LIST);
    }
    const_iterator cbegin()
    {
        return const_iterator(list_, list_.cbegin(), Behaviour::LIST);
    }
    iterator end() { return iterator(list_, list_.end(), Behaviour::LIST); }
    const_iterator end() const
    {
        return const_iterator(list_, list_.end(), Behaviour::LIST);
    }
    const_iterator cend()
    {
        return const_iterator(list_, list_.cend(), Behaviour::LIST);
    }
    iterator citer() { return iterator(list_, list_.begin()); }
    const_iterator citer() const { return const_iterator(list_, list_.begin()); }

    // Modifiers
    void clear() { list_.clear(); }
    iterator insert(const_iterator pos, const T &value)
    {
        auto cit = list_.insert(pos.cit_, value);
        return iterator(list_, cit);
    }
    iterator insert(const_iterator pos, T &&value)
    {
        auto cit = list_.insert(pos.cit_, std::move(value));
        return iterator(list_, cit);
    }
    iterator insert(const_iterator pos, size_type count, const T &value)
    {
        auto cit = list_.insert(pos.cit_, count, value);
        return iterator(list_, cit);
    }
    template <typename InputIt>
    iterator insert(const_iterator pos, InputIt first, InputIt last)
    {
        auto it = list_.insert(pos.cit_, first, last);
        return iterator(list_, it);
    }
    iterator insert(const_iterator pos, std::initializer_list<T> ilist)
    {
        auto it = list_.insert(pos.cit_, ilist);
        return iterator(list_, it);
    }
    void push_front(const T &value) { list_.push_front(value); }
    void push_front(T &&value) { list_.push_front(std::move(value)); }
    void push_back(const T &value) { list_.push_back(value); }
    void push_back(T &&value) { list_.push_back(std::move(value)); }
    void assign(size_type count, const T &value) { list_.assign(count, value); }
    template <typename InputIt>
    void assign(InputIt first, InputIt last)
    {
        list_.assign(first, last);
    }
    void assign(std::initializer_list<T> ilist) { list_.assign(ilist); }
    template <class... Args>
    iterator emplace(const_iterator pos, Args &&... args)
    {
        auto it = list_.emplace(pos, std::forward<Args>(args)...);
        return iterator(list_, it);
    }
    template <class... Args>
    iterator emplace_back(Args &&... args)
    {
        list_.emplace_back(std::forward<Args>(args)...);
        return iterator(list_, --list_.end());
    }
    template <class... Args>
    iterator emplace_front(Args &&... args)
    {
        list_.emplace_front(std::forward<Args>(args)...);
        return iterator(list_, list_.begin());
    }
    void resize(size_type count) { list_.resize(count, T()); }
    void resize(size_type count, const T &value) { list_.resize(count, value); }
    iterator erase(const_iterator pos)
    {
        auto it = list_.erase(pos.cit_);
        return it == list_.end() ? iterator(list_, list_.begin())
                                 : iterator(list_, it);
    }
    iterator erase(const_iterator first, const_iterator last)
    {
        auto it = list_.erase(first.cit_, last.cit_);
        return it == list_.end() ? iterator(list_, list_.begin())
                                 : iterator(list_, it);
    }
    void pop_front() { list_.pop_front(); }
    void pop_back() { list_.pop_back(); }
    void swap(CircularList &other) noexcept { list_.swap(other.list_); }
    void splice(const_iterator pos, CircularList &&clist)
    {
        list_.splice(pos.cit_, std::move(clist.list_));
    }
    void splice(const_iterator pos, CircularList &&clist, const_iterator it)
    {
        list_.splice(pos.cit_, std::move(clist.list_), it.cit_);
    }

  private:
    std::list<T> list_;
};

template <typename T>
bool operator==(const CircularList<T> &cla, const CircularList<T> &clb)
{
    return std::equal(cla.begin(), cla.end(), clb.begin(), clb.end());
}

template <typename T>
bool operator!=(const CircularList<T> &cla, const CircularList<T> &clb)
{
    return !(cla == clb);
}

template <typename T>
bool operator<(const CircularList<T> &cla, const CircularList<T> &clb)
{
    return std::lexicographical_compare(cla.begin(), cla.end(), clb.begin(),
                                        clb.end());
}

template <typename T>
bool operator>(const CircularList<T> &cla, const CircularList<T> &clb)
{
    return std::lexicographical_compare(cla.begin(), cla.end(), clb.begin(),
                                        clb.end(), std::greater<T>());
}

template <typename T>
bool operator<=(const CircularList<T> &cla, const CircularList<T> &clb)
{
    return !(cla > clb);
}

template <typename T>
bool operator>=(const CircularList<T> &cla, const CircularList<T> &clb)
{
    return !(cla < clb);
}

#endif
