# frozen_string_literal: true
# For each non-prefix lost frame k: was k-1 received beside this node's
# own completion (TXFRS in its status word)? Per host and placement.
dir = ARGV[0]; first = Integer(ARGV[1] || 0); last = Integer(ARGV[2] || 99_999)
TXFRS = 1 << 7
st = Hash.new { |h, k| h[k] = Hash.new(0) }
detail = []
Dir["#{dir}/run-*-rpi-?.log"].sort.each do |f|
  run, host = f.match(/run-(\d+)-(rpi-.)/).captures; run = run.to_i
  next if run < first || run > last
  rx = {}
  File.foreach(f) { |l| next unless l.start_with?("RXSEQ"); a = l.split; rx[a[1].to_i] ||= a[4].to_i(16) }
  next if rx.empty?
  key = [host, run.odd? ? "early" : "end"]
  s = st[key]
  s[:runs] += 1
  s[:frames] += rx.size
  s[:joint]  += rx.values.count { _1.anybits?(TXFRS) }
  missing = (0...50).to_a - rx.keys
  lead = missing.take_while.with_index { |m, i| m == i }.size
  missing.drop(lead).each do |k|
    s[:lost] += 1
    prev = rx[k - 1]
    kind = if prev.nil? then :prev_lost
           elsif prev.anybits?(TXFRS) then :prev_joint
           else :prev_plain end
    s[kind] += 1
    s[:next_ok] += 1 if rx[k + 1]
    detail << [run, host, key[1], k, kind, format("0x%08x", prev || 0)]
  end
  # frames received after a joint frame: how many of those k+1 were received
  rx.each { |k, w| next unless w.anybits?(TXFRS); next if k == 49; s[:after_joint] += 1; s[:after_joint_lost] += 1 unless rx[k + 1] }
end
st.sort.each do |(h, p), s|
  printf "%-6s %-5s runs %3d frames %5d joint %3d | lost %3d: prev_joint %3d prev_plain %3d prev_lost %3d next_ok %3d | after a joint frame: %3d, next lost %3d (%.1f%%)\n",
    h, p, s[:runs], s[:frames], s[:joint], s[:lost], s[:prev_joint], s[:prev_plain], s[:prev_lost], s[:next_ok], s[:after_joint], s[:after_joint_lost], 100.0 * s[:after_joint_lost] / [s[:after_joint], 1].max
end
if ENV["DETAIL"]
  detail.each { |d| puts d.join(" ") }
end
