# frozen_string_literal: true
# Per run: fit peer-tx -> own-rx chip clocks on the received frames, place
# every lost frame's arrival in the loser's clock, and measure its offset
# from the loser's nearest own send (RMARKER to RMARKER, in us).
# Also: loss episodes (maximal runs of consecutive lost seqs).
dir = ARGV[0]; first = Integer(ARGV[1] || 0); last = Integer(ARGV[2] || 99_999)
WRAP = 1 << 40; US = 499.2e6 * 128 / 1e6
def unwrap(x) = x >= WRAP / 2 ? x - WRAP : x
def parse(f)
  rx = {}; tx = {}
  File.foreach(f) do |l|
    a = l.split
    case a[0]
    when "RXSEQ" then rx[a[1].to_i] ||= { t: a[2].to_f, ts: a[3].to_i, st: a[4].to_i(16) }
    when "TXSEQ" then tx[a[1].to_i] ||= a[2].to_i
    end
  end
  [rx, tx]
end
runs = Dir["#{dir}/run-*-rpi-c.log"].map { |f| f[/run-(\d+)/, 1].to_i }.select { _1.between?(first, last) }.sort
ep = Hash.new { |h, k| h[k] = { runs: 0, lossy_runs: 0, lost: 0, episodes: 0, lens: [] } }
lost_rows = []
hist = Hash.new { |h, k| h[k] = Hash.new(0) }
runs.each do |run|
  logs = { "rpi-c" => parse("#{dir}/run-#{format('%03d', run)}-rpi-c.log"),
           "rpi-d" => parse("#{dir}/run-#{format('%03d', run)}-rpi-d.log") }
  pl = run.odd? ? "early" : "end"
  logs.each do |host, (rx, tx)|
    peer = host == "rpi-c" ? "rpi-d" : "rpi-c"
    _prx, ptx = logs[peer]
    next if rx.empty? || tx.empty?
    e = ep[[host, pl]]
    e[:runs] += 1
    missing = (0...50).to_a - rx.keys
    lead = missing.take_while.with_index { |m, i| m == i }.size
    scattered = missing.drop(lead)
    e[:lost] += scattered.size
    e[:lossy_runs] += 1 unless scattered.empty?
    scattered.slice_when { |a, b| b != a + 1 }.each { |g| e[:episodes] += 1; e[:lens] << g.size }
    # clock fit: own rx rmarker of seq k vs peer tx rmarker of seq k
    pairs = rx.keys.select { ptx[_1] }.map { |k| [ptx[k].to_f, unwrap(rx[k][:ts] - ptx[k]).to_f] }
    next if pairs.size < 5
    med = pairs.map(&:last).sort[pairs.size / 2]
    dropped = pairs.count { |_, y| (y - med).abs > 5e6 }
    pairs = pairs.reject { |_, y| (y - med).abs > 5e6 }
    warn "run #{run} #{host}: #{dropped} pair(s) off the clock relation dropped" if dropped > 0
    n = pairs.size; sx = pairs.sum(&:first); sy = pairs.sum(&:last)
    sxx = pairs.sum { |x, _| x * x }; sxy = pairs.sum { |x, y| x * y }
    b = (n * sxy - sx * sy) / (n * sxx - sx * sx); a = (sy - b * sx) / n
    resid = pairs.map { |x, y| (y - (a + b * x)) / US }
    rms = Math.sqrt(resid.sum { _1 * _1 } / n)
    own = tx.values.sort
    place = lambda do |k|
      return nil unless ptx[k]
      arr = ptx[k] + a + b * ptx[k]          # arrival RMARKER in own clock
      d = own.map { |o| (arr - o) / US }.min_by(&:abs)
      d
    end
    rx.each_key { |k| d = place.(k) and hist[[host, "rx"]][(d / 250).floor * 250] += 1 }
    scattered.each do |k|
      d = place.(k)
      lost_rows << [run, host, pl, k, d&.round(1), rms.round(3), rx[k - 1] ? "" : "prev-lost", rx[k + 1] ? "" : "next-lost"]
      hist[[host, "lost-#{pl}"]][d ? (d / 250).floor * 250 : :nofit] += 1
    end
  end
end
puts "== episodes (unit: a maximal run of consecutive lost seqs, prefix removed)"
ep.sort.each do |(h, p), e|
  printf "%-6s %-5s runs %3d  lossy runs %2d  lost %3d  episodes %3d  episode lengths %s\n", h, p, e[:runs], e[:lossy_runs], e[:lost], e[:episodes], e[:lens].sort.reverse.inspect
end
puts "== lost frames: offset (us) of the lost frame's RMARKER from the loser's nearest own TX RMARKER (negative: peer's frame first)"
lost_rows.sort_by { |r| [r[1], r[2], r[0], r[3]] }.each { |r| puts r.join("\t") }
puts "== histogram of offsets, 250 us bins, |offset| <= 3000 us shown; 'rx' = received frames (both placements)"
hist.sort_by { |k, _| k }.each do |(h, kind), hh|
  near = hh.select { |b, _| b.is_a?(Integer) && b.abs <= 3000 }.sort
  far  = hh.sum { |b, c| b.is_a?(Integer) && b.abs > 3000 ? c : 0 }
  puts "#{h} #{kind}: far(#{far}) " + near.map { |b, c| "#{b}:#{c}" }.join(" ")
end
puts "== far losses (|offset| > 500 us, seq 49 excluded) per host/placement"
lost_rows.group_by { |r| [r[1], r[2]] }.sort.each do |k, rs|
  far = rs.select { |r| r[4] && r[4].abs > 500 && r[3] != 49 }
  puts "#{k.join(' ')}: far #{far.size} of #{rs.size}  in runs #{far.map(&:first).uniq.inspect}"
end
