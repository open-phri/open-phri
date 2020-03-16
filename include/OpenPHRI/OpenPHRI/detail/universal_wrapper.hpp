#pragma once

#include <memory>
#include <variant>

namespace phri {
namespace detail {

template <typename ValueT> class UniversalWrapper {
public:
    template <typename T> explicit UniversalWrapper(T&& value) noexcept {
        using OtherT = decltype(value);
        constexpr bool is_valid_shared_pointer =
            std::is_same_v<std::decay_t<OtherT>, std::shared_ptr<ValueT>>;

        static_assert(
            is_valid_shared_pointer or
            std::is_same_v<std::remove_const_t<
                               std::remove_pointer_t<std::decay_t<OtherT>>>,
                           ValueT>);

        // For rvalue references to ValueT
        if constexpr (std::is_rvalue_reference_v<OtherT>) {
            value_ = std::move(value);
        }
        // For const pointers to ValueT
        else if constexpr (std::is_same_v<const ValueT*,
                                          std::remove_reference_t<OtherT>>) {
            value_ = const_cast<ValueT*>(value);
        }
        // For (possibly const) lvalue references to ValueT
        else if constexpr (std::is_lvalue_reference_v<OtherT> and
                           not is_valid_shared_pointer) {
            value_ = const_cast<ValueT*>(std::addressof(value));
        }
        // For values, pointers and shared_ptr to ValueT
        else {
            value_ = value;
        }
    }

    UniversalWrapper() noexcept : value_{ValueT{}} {
    }

    ValueT& value() noexcept {
        if (std::holds_alternative<ValueT>(value_)) {
            return std::get<ValueT>(value_);
        } else if (std::holds_alternative<ValueT*>(value_)) {
            return *std::get<ValueT*>(value_);
        } else {
            return *std::get<std::shared_ptr<ValueT>>(value_);
        }
    }

    const ValueT& value() const noexcept {
        if (std::holds_alternative<ValueT>(value_)) {
            return std::get<ValueT>(value_);
        } else if (std::holds_alternative<ValueT*>(value_)) {
            return *std::get<ValueT*>(value_);
        } else {
            return *std::get<std::shared_ptr<ValueT>>(value_);
        }
    }

    operator ValueT&() noexcept {
        return value();
    }

    operator const ValueT&() const noexcept {
        return value();
    }

private:
    std::variant<ValueT, std::shared_ptr<ValueT>, ValueT*> value_;
};

} // namespace detail
} // namespace phri