# frozen_string_literal: true
# Each non-prefix lost frame in the traced runs, by the pass that stood
# where its rx_ok should have: an RXOVRR error, a detect-only completion
# (RXPRD|RXSFDD|RXPHD beside TXFRS, no RXFCG), or neither.
dir = ARGV[0]; first = Integer(ARGV[1] || 0); last = Integer(ARGV[2] || 99_999)
B = ->(n) { 1 << n }
DETECT = B[8] | B[9] | B[11]; RXFCG = B[14]; RXDFR = B[13]; TXFRS = B[7]; RXOVRR = B[20]
ERR = B[12] | B[15] | B[16] | B[17] | B[18] | B[21] | B[26] | B[29]
st = Hash.new { |h, k| h[k] = Hash.new(0) }
Dir["#{dir}/run-*-rpi-?.log"].sort.each do |f|
  run, host = f.match(/run-(\d+)-(rpi-.)/).captures; run = run.to_i
  next if run < first || run > last
  lines = File.readlines(f)
  next unless lines.any? { _1.start_with?("PASS") }
  s = st[[host, run.odd? ? "early" : "end"]]
  first_rx = {}   # seq -> line index of its first RXSEQ
  lines.each_with_index { |l, i| next unless l.start_with?("RXSEQ"); k = l.split[1].to_i; first_rx[k] ||= i }
  missing = (0...50).to_a - first_rx.keys
  lead = missing.take_while.with_index { |m, i| m == i }.size
  missing.drop(lead).each do |k|
    s[:lost] += 1
    a = first_rx.select { |q, _| q < k }.values.max || 0
    b = first_rx.select { |q, _| q > k }.values.min || lines.size
    words = lines[a..b].grep(/^PASS/).map { |l| ev, *w = l.split[1..]; w.unshift(ev) if ev.start_with?("0x"); [ev, (w[0] || "0").to_i(16)] }
    kind = if words.any? { |_, w| (w & RXOVRR) != 0 } then :overrun
           elsif words.any? { |ev, w| ev == "tx_done" && (w & DETECT) != 0 && (w & (RXFCG | RXDFR | ERR)).zero? } then :detect_only
           elsif words.any? { |_, w| (w & ERR) != 0 } then :other_error
           else :nothing end
    s[kind] += 1
  end
end
st.sort.each { |(h, p), s| printf "%-6s %-5s lost %3d: overrun %3d  detect-only %3d  other-error %3d  no trace (dead window) %3d\n", h, p, s[:lost], s[:overrun], s[:detect_only], s[:other_error], s[:nothing] }
