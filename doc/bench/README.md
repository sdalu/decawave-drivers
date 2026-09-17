# dblbuff bench, 2026-09-17

Raw responder output from the run recorded in AUDIT.md under "Settled: a
single-buffered responder loses the REPORT, every time".

rpi-c initiator, rpi-d responder, channel 5, probe built from
1.3.1+60.ga345bb9. Four rounds, round-robin over the four combinations so
drift falls on all four equally, 30 exchanges each.

File names are `r<round>-<initiator><responder>.log`, where `D` is
`--dblbuff` and `S` is `--no-dblbuff`. Each file is one responder run:
its `TWR` record lines, then a `STATS` line.

```sh
grep -h '^TWR' r*-DD.log | grep -o 'asym_mm=[0-9.]*'
```

The responder emits the records, not the initiator: it is the end that
holds all six timestamps once the REPORT lands, which is exactly what the
single-buffered runs never get.
