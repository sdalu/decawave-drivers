# frozen_string_literal: true
# Stale passes (rx_ok,tx_done with the two buffer pointers aligned and no
# RXPRD/RXSFDD/RXPHD on entry) and what became of the peer's next frames;
# detect-only completion passes; overruns. Per host and placement.
dir = ARGV[0]; first = Integer(ARGV[1] || 0); last = Integer(ARGV[2] || 99_999)
B = ->(n) { 1 << n }
PRD, SFDD, PHD, RXDFR, RXFCG, TXFRS, RXOVRR, HS, IC = B[8], B[9], B[11], B[13], B[14], B[7], B[20], B[30], B[31]
DETECT = PRD | SFDD | PHD
ERR = B[12] | B[15] | B[16] | B[17] | B[18] | B[21] | B[26] | B[29]
def aligned(w) = ((w & HS) != 0) == ((w & IC) != 0)
st = Hash.new { |h, k| h[k] = Hash.new(0) }
Dir["#{dir}/run-*-rpi-?.log"].sort.each do |f|
  run, host = f.match(/run-(\d+)-(rpi-.)/).captures; run = run.to_i
  next if run < first || run > last
  lines = File.readlines(f)
  next unless lines.any? { _1.start_with?("PASS") }
  s = st[[host, run.odd? ? "early" : "end"]]
  s[:runs] += 1
  rx = {}
  lines.each { |l| next unless l.start_with?("RXSEQ"); a = l.split; rx[a[1].to_i] ||= a[4].to_i(16) }
  lines.each_with_index do |l, i|
    next unless l.start_with?("PASS")
    ev, *w = l.split[1..]
    if ev.start_with?("0x") then w.unshift(ev); ev = "" end
    w = w.map { _1.to_i(16) }
    e = w[0] || 0
    s[:passes] += 1
    s[:joint] += 1 if ev == "rx_ok,tx_done"
    if ev == "rx_ok,tx_done" && aligned(e) && (e & DETECT).zero?
      s[:stale] += 1
      # the next peer seq after this pass: the RXSEQ lines after i give the seqs already delivered;
      # take the max seq delivered before this line (excluding dup re-deliveries) and look at max+1, max+2
      before = lines[0...i].grep(/^RXSEQ/).map { _1.split[1].to_i }
      nxt = (before.max || -1) + 1
      [nxt, nxt + 1].each_with_index do |k, j|
        next if k > 49
        s[["after_stale_#{j + 1}", rx[k] ? :rx : :lost]] += 1
      end
    end
    s[:detect_only_txfrs] += 1 if ev == "tx_done" && (e & TXFRS) != 0 && (e & DETECT) != 0 && (e & (RXFCG | RXDFR | ERR)).zero?
    s[:overrun] += 1 if (e & RXOVRR) != 0
    s[:rx_error] += 1 if ev.include?("rx_error")
    s[:err_bits] += 1 if (e & ERR) != 0
  end
end
st.sort.each do |(h, p), s|
  printf "%-6s %-5s runs %2d passes %5d joint %3d stale %3d | after a stale pass: next rx %3d lost %3d, next+1 rx %3d lost %3d | detect-only TXFRS passes %3d | rx_error %3d (RXOVRR %3d, other err bits %3d)\n",
    h, p, s[:runs], s[:passes], s[:joint], s[:stale],
    s[["after_stale_1", :rx]], s[["after_stale_1", :lost]], s[["after_stale_2", :rx]], s[["after_stale_2", :lost]],
    s[:detect_only_txfrs], s[:rx_error], s[:overrun], s[:err_bits]
end
