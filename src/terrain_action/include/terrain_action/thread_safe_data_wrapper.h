#include <mutex>
#include <utility>

namespace terrain_action
{
template <class T>
class Thread_Data_Wrapper
{
public:
  Thread_Data_Wrapper() = default;

  Thread_Data_Wrapper(const Thread_Data_Wrapper& other_data) : data_(other_data.getData())
  {
  }

  void setData(T data)
  {
    std::lock_guard<std::mutex> _{ mutex_ };
    data_ = data;
  }

  T getData() const
  {
    std::lock_guard<std::mutex> _{ mutex_ };
    return data_;
  };

  std::pair<T &, std::lock_guard<std::mutex>> getDataToWrite()
  {
    std::unique_lock<std::mutex> data_write_guard{ mutex_ };
    return {data_, std::move(data_write_guard)};
  }

private:
  T data_;
  mutable std::mutex mutex_;
};
}  // namespace terrain_action
