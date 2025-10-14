# Sample Unit Tests Using the Google Test Framework

## Files
- `sample_src.c`, `sample_src.h`: The source code to be tested.
- `sample_sub.h`: The header file for the module which sample_src.c depends on
- `sample_mock.cc`, `sample_mock.h`: The mock of the `SampleSub` function.
- `sample_gtest.cc`: The unit tests

## Note
- In the actual unit tests, `sample_src.c`, `sample_src.h`, and `sample_sub.h` should be located under `src/` directory
