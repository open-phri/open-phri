#pragma once

#include <memory>
#include <variant>

namespace phri {
namespace detail {

template <typename T> struct FinalTypeImpl {
    using type = std::decay_t<std::remove_pointer_t<std::decay_t<T>>>;
};

template <typename T> struct FinalTypeImpl<std::shared_ptr<T>> {
    using type = std::remove_const_t<typename T::element_type>;
};

template <class T> using FinalType = typename FinalTypeImpl<T>::type;

template <typename ValueT> class UniversalWrapper {
public:
    template <typename T> explicit UniversalWrapper(T&& value) noexcept {
        using OtherT = decltype(value);
        using NonConstValueT = std::remove_const_t<ValueT>;
        using ConstValueT = std::add_const_t<NonConstValueT>;
        using NonConstFinalT = FinalType<OtherT>;
        using ConstFinalT = std::add_const_t<FinalType<OtherT>>;

        constexpr bool is_valid_shared_pointer =
            std::is_same_v<std::decay_t<OtherT>,
                           std::shared_ptr<NonConstValueT>> or
            std::is_same_v<std::decay_t<OtherT>, std::shared_ptr<ConstValueT>>;

        static_assert(is_valid_shared_pointer or
                      std::is_same_v<NonConstFinalT, NonConstValueT>);

        if constexpr (std::is_lvalue_reference_v<OtherT> and
                      not std::is_pointer_v<std::remove_reference_t<OtherT>> and
                      not is_valid_shared_pointer) {
            value_ = std::addressof(value);
        } else if constexpr (std::is_move_assignable_v<OtherT> and
                             not is_valid_shared_pointer) {
            value_ = std::move(value);
        } else {
            value_ = value;
        }
    }

    UniversalWrapper() noexcept : value_{ValueT{}} {
    }

    template <typename T = ValueT>
    [[nodiscard]] typename std::enable_if_t<not std::is_const_v<T>, T&> ref() {
        if (std::holds_alternative<ValueT>(value_)) {
            return std::get<ValueT>(value_);
        } else if (std::holds_alternative<ValueT*>(value_)) {
            return *std::get<ValueT*>(value_);
        } else if (std::holds_alternative<std::shared_ptr<ValueT>>(value_)) {
            return *std::get<std::shared_ptr<ValueT>>(value_);
        } else {
            throw std::logic_error("UniversalWrapper: cannot provide a "
                                   "non-const reference to a const value");
        }
    }

    [[nodiscard]] std::add_const_t<ValueT>& cref() const noexcept {
        using NonConstValueT = std::remove_const_t<ValueT>;
        using ConstValueT = std::add_const_t<NonConstValueT>;
        if (std::holds_alternative<NonConstValueT>(value_)) {
            return std::get<NonConstValueT>(value_);
        } else if (std::holds_alternative<NonConstValueT*>(value_)) {
            return *std::get<NonConstValueT*>(value_);
        } else if (std::holds_alternative<std::shared_ptr<NonConstValueT>>(
                       value_)) {
            return *std::get<std::shared_ptr<NonConstValueT>>(value_);
        } else if (std::holds_alternative<ConstValueT>(value_)) {
            return std::get<ConstValueT>(value_);
        } else if (std::holds_alternative<ConstValueT*>(value_)) {
            return *std::get<ConstValueT*>(value_);
        } else {
            return *std::get<std::shared_ptr<ConstValueT>>(value_);
        }
    }

    template <typename T = ValueT>
    [[nodiscard]]
    operator typename std::enable_if_t<not std::is_const_v<T>, T&>() noexcept {
        return ref();
    }

    [[nodiscard]] operator const ValueT&() const noexcept {
        return cref();
    }

    [[nodiscard]] bool operator==(const UniversalWrapper<ValueT>& other) {
        return cref() == other.cref();
    }

    [[nodiscard]] bool operator!=(const UniversalWrapper<ValueT>& other) {
        return not(*this == other);
    }

    [[nodiscard]] bool operator>(const UniversalWrapper<ValueT>& other) {
        return cref() > other.cref();
    }

    [[nodiscard]] bool operator>=(const UniversalWrapper<ValueT>& other) {
        return cref() >= other.cref();
    }

    [[nodiscard]] bool operator<(const UniversalWrapper<ValueT>& other) {
        return cref() < other.cref();
    }

    [[nodiscard]] bool operator<=(const UniversalWrapper<ValueT>& other) {
        return cref() <= other.cref();
    }

private:
    std::variant<std::monostate, std::remove_cv_t<ValueT>,
                 std::shared_ptr<std::remove_cv_t<ValueT>>,
                 std::remove_cv_t<ValueT>*, const ValueT,
                 std::shared_ptr<const ValueT>, const ValueT*>
        value_;
};

} // namespace detail
} // namespace phri