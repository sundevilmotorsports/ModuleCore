#pragma once

#include "esp_err.h"
#include <string_view>
#include <cstdint>

class ModuleCoreError {
public:
    enum class ErrorType : uint8_t {
        Ok = 0,
        MainCrash = 1,
    };

private:
    static constexpr std::string_view error_message_for(ErrorType e) noexcept {
        switch (e) {
            case ErrorType::Ok: return "Ok";
            case ErrorType::MainCrash: return "Main function crashed";
        }
        return "Unknown error";
    }

public:
    constexpr ModuleCoreError() noexcept : is_esp_(false), num_(0), message_(error_message_for(ErrorType::Ok)) {}
    constexpr ModuleCoreError(ErrorType err) noexcept : is_esp_(false), num_(static_cast<uint8_t>(err)), message_(error_message_for(err)) {}
    explicit ModuleCoreError(esp_err_t err) noexcept : is_esp_(true), num_(static_cast<uint8_t>(err)), message_(esp_err_to_name(err)) {}

    constexpr bool operator==(const ModuleCoreError &a) const noexcept { return is_esp_ == a.is_esp_ && num_ == a.num_; }
    constexpr bool operator!=(const ModuleCoreError &a) const noexcept { return !(*this == a); }

    constexpr bool is_esp() const noexcept { return is_esp_; }
    constexpr int  get_num() const noexcept { return num_; }
    constexpr bool is_ok() const noexcept { return num_ == 0; }
    constexpr std::string_view get_message() const noexcept { return message_; }

private:
    bool                is_esp_  = false;
    int                 num_     = 0;
    std::string_view    message_ = "";
};