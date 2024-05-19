# zephyr_on_xxx
## Various SoC  running Zephyr RTOS

### Steps
- Clone repo
  ```
  git clone https://github.com/zephyr_on_xxx.git
  ```

- cmake usage:
  ```
  cd zephyr_on_xxx
  cmake -B build xxx/licheerv_nano_c906b_test
  make -C build
  ```

- west usage:
  ```
  cd zephyr_on_xxx
  west build -s xxx/licheerv_nano_c906b_test -t menuconfig -b licheerv_nano_c906b
  west build -p -s xxx/licheerv_nano_c906b_test -b licheerv_nano_c906b
  ```

