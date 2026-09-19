# frozen_string_literal: true
# Soak logs with RXSEQ/TXSEQ traces: per node and placement, the
# frames lost as a leading prefix (start skew) and otherwise; the
# duplicates and whether they share a timestamp; the passes that
# carried a good frame beside a receive error or timeout.
dir   = ARGV[0]
first = Integer(ARGV[1] || 0)
last  = Integer(ARGV[2] || 10_000)
ERR   = (1 << 12) | (1 << 16) | (1 << 18) | (1 << 29) | (1 << 26) | (1 << 17) | (1 << 21) | (1 << 15)
RXFCG = 1 << 14
stats = Hash.new { |h, k| h[k] = { runs: 0, prefix: 0, other: 0, dup_same_ts: 0, dup_diff_ts: 0, fcg_err: 0, frames: 0, other_list: [] } }
Dir["#{dir}/run-*-rpi-?.log"].sort.each do |f|
    run, host = f.match(/run-(\d+)-(rpi-.)/).captures
    run = run.to_i
    next if run < first || run > last
    lines = File.readlines(f)
    rx = lines.grep(/^RXSEQ/).map { |l| a = l.split; [ a[1].to_i, a[3]&.to_i, a[4]&.to_i(16) ] }
    next if rx.empty?
    s = stats[[ host, run.odd? ? "early" : "end" ]]
    s[:runs] += 1
    seqs = rx.map(&:first)
    missing = (0...50).to_a - seqs
    lead = missing.take_while.with_index { |m, i| m == i }.size
    s[:prefix] += lead
    s[:other]  += missing.size - lead
    s[:other_list] << (missing.size - lead)
    rx.group_by(&:first).each_value do |g|
        next if g.size < 2
        g.map { _1[1] }.uniq.size == 1 ? s[:dup_same_ts] += g.size - 1 : s[:dup_diff_ts] += g.size - 1
    end
    s[:frames]  += rx.size
    s[:fcg_err] += rx.count { |_, _, st| st && st.anybits?(RXFCG) && st.anybits?(ERR) }
end
stats.sort.each do |(h, p), s|
    ol = s[:other_list].sort
    printf "%-6s %-5s runs %2d  prefix-lost %3d  other-lost %3d (per run mean %.2f median %d max %d)  dup same-ts %d diff-ts %d  RXFCG+err passes %d/%d\n",
           h, p, s[:runs], s[:prefix], s[:other], s[:other].to_f / s[:runs], ol[ol.size / 2], ol.max, s[:dup_same_ts], s[:dup_diff_ts], s[:fcg_err], s[:frames]
end
