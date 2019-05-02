#pragma once

#include <OpenPHRI/type_aliases.h>
#include <OpenPHRI/definitions.h>

#include <fstream>
#include <string>
#include <sstream>
#include <type_traits>

namespace phri {

enum class SkipRows : size_t {};
enum class SkipCols : size_t {};

template <typename ArrayT> class DataReplayer {
public:
    DataReplayer(const std::string& file_path, ArrayT& array, size_t size,
                 SkipRows skip_rows = SkipRows(0),
                 SkipCols skip_cols = SkipCols(0))
        : array_(array),
          size_(size),
          rows_to_skip_(static_cast<size_t>(skip_rows)),
          cols_to_skip_(static_cast<size_t>(skip_cols)) {
        init(file_path);
    }

    ~DataReplayer() {
        log_file_.close();
    }

    bool process() {
        std::string line;

        // Return false if the end of the file is reached
        if (not readLine(line)) {
            return false;
        }

        std::stringstream ss(line);
        std::string item;
        int i = 0;

        size_t columns_to_skip = cols_to_skip_;
        while (std::getline(ss, item, '\t') and i < size_) {
            if (columns_to_skip > 0) {
                --columns_to_skip;
                continue;
            }
            set(i++, atof(item.c_str()));
        }
        return true;
    }

    bool operator()() {
        return process();
    }

private:
    void init(const std::string& file_path) {
        log_file_.open(file_path.c_str());

        if (log_file_.is_open()) {
            std::string line;
            for (int i = 0; i < rows_to_skip_; ++i) {
                if (not readLine(line))
                    break;
            }
        } else {
            throw std::runtime_error(OPEN_PHRI_ERROR(
                "Cannot open the file " + file_path + " for reading"));
        }
    }

    bool readLine(std::string& line) {
        return static_cast<bool>(std::getline(log_file_, line));
    }

    template <typename T>
    using element_type_t =
        std::remove_reference_t<decltype(std::declval<T>()[0])>;

    template <typename T = ArrayT>
    void
    set(size_t index, element_type_t<typename T::element_type> value,
        typename std::enable_if<is_shared_ptr<T>::value>::type* = nullptr) {
        (*array_)[index] = value;
    }

    template <typename T = ArrayT>
    void
    set(size_t index, element_type_t<T> value,
        typename std::enable_if<not is_shared_ptr<T>::value>::type* = nullptr) {
        array_[index] = value;
    }

    ArrayT& array_;
    size_t size_;
    size_t rows_to_skip_;
    size_t cols_to_skip_;
    std::ifstream log_file_;
};

} // namespace phri
