# `NUCLEO-F411RE` playground

[`probe-run`] + [`defmt`] + [`flip-link`] + [`rtic`] Rust embedded playground

[`probe-run`]: https://crates.io/crates/probe-run
[`defmt`]: https://github.com/knurling-rs/defmt
[`flip-link`]: https://github.com/knurling-rs/flip-link
[`rtic`]: https://github.com/rtic-rs/cortex-m-rtic

## Dependencies

#### 1. `flip-link`:

```console
$ cargo install flip-link
```

#### 2. `probe-run`:

```console
$ cargo install probe-run
```

## Run!

Start by `cargo run`-ning `src/bin/exti.rs`:

```console
$ # `rb` is an alias for `run --bin`
$ DEFMT_LOG=info cargo rb exti
  Finished dev [optimized + debuginfo] target(s) in 0.3s
  Running `probe-run --chip STM32F411RETx target/thumbv7em-none-eabihf/debug/exti`
  (HOST) INFO  flashing program (13.39 KiB)
  (HOST) INFO  success!
────────────────────────────────────────────────────────────────────────────────
0.000000 INFO  Press button!
└─ exti::init @ src/bin/exti.rs:38
(..)
```

## Run hardware tests

Conenct pin PA8 to GND and PB9 to Vdd then run `cargo test` from inside the `testsuite` folder

```console
$ cargo test
    Finished test [optimized + debuginfo] target(s) in 0.03s
     Running /GIT/f411-rtic/target/thumbv7em-none-eabihf/debug/deps/gpio-4ace02b17c13e985
  (HOST) INFO  flashing program (11.00 KiB)
  (HOST) INFO  success!
────────────────────────────────────────────────────────────────────────────────
0.000000 INFO  (1/2) running `gnd_is_low`...
└─ gpio::tests::__defmt_test_entry @ tests/gpio.rs:20
0.000001 INFO  (2/2) running `vdd_is_high`...
└─ gpio::tests::__defmt_test_entry @ tests/gpio.rs:20
0.000002 INFO  all tests passed!
(..)
```
