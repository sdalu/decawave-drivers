# frozen_string_literal: true
# For each duplicate delivery in traced runs (RXSEQ with the same seq and
# rmarker), the passes between the two copies with their three status
# words: on entry, after the double buffered clear, after the toggle.
# Bits: RXFCG 14, RXDFR 13, LDEDONE 10, RXPRD 8, TXFRB 4, TXFRS 7,
# HSRBP 30, ICRBP 31.
def bits(w)
  return "?" if w.nil?
  return "-" if w.zero?
  f = []
  f << "FCG" if w[14] == 1; f << "DFR" if w[13] == 1; f << "LDE" if w[10] == 1
  f << "PRD" if w[8] == 1;  f << "TXFRB" if w[4] == 1; f << "TXFRS" if w[7] == 1
  format("%s H%d I%d", f.join("|").ljust(24), w[30], w[31])
end
dir, first, last = ARGV[0], Integer(ARGV[1] || 0), Integer(ARGV[2] || 99_999)
ndup = 0; shapes = Hash.new(0)
Dir["#{dir}/run-*-rpi-?.log"].sort.each do |f|
  run, host = f.match(/run-(\d+)-(rpi-.)/).captures
  next if run.to_i < first || run.to_i > last
  lines = File.readlines(f)
  idx = {}
  lines.each_with_index do |l, i|
    next unless l.start_with?("RXSEQ")
    seq, _t, ts = l.split[1..3]
    (idx[[seq, ts]] ||= []) << i
  end
  idx.select { |_, v| v.size > 1 }.sort_by { |_, v| v.first }.each do |(seq, _ts), (a, b)|
    ndup += 1
    puts "== run #{run} #{host} seq #{seq} delivered twice"
    lines[a..b].each do |l|
      if l.start_with?("PASS")
        _p, ev, *w = l.split
        w = w.map { _1.to_i(16) }
        puts format("   PASS %-18s entry %-32s clear %-32s toggle %s", ev, bits(w[0]), bits(w[1]), bits(w[2]))
      elsif l.start_with?("RXSEQ")
        puts "   #{l.strip}"
      end
    end
    inflight = lines[a..b].grep(/^PASS/).find { |l| f = l.split; f.size >= 5 && (w = f[2].to_i(16)) && w[4] == 1 && w[7] == 0 && w[14] == 1 }
    if inflight
      w = inflight.split[2..].map { _1.to_i(16) }
      shapes[[w[1][14], w[2][14]]] += 1
    end
  end
end
puts "duplicates: #{ndup}; in-flight pass shapes [RXFCG after clear, RXFCG after toggle] => #{shapes.inspect}"
