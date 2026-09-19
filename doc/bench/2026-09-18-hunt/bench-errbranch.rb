# frozen_string_literal: true
# The error branch on the chip: rpi-c counts a 3000-frame flood from
# rpi-d, once alone (baseline) and once with rpi-b flooding a foreign
# prefix at the same time so that collisions raise CRC and header errors
# on rpi-c; the receiver must come back from every one of them. With
# DW1000_DUPLEX_TRACE=1 the count role logs every event pass, so the
# passes that took the error and timeout branches are counted, and the
# receptions after the last error say whether the receiver came back.
# Usage: errbranch.rb [jam] [OUTDIR]
# DECAWAVE_DRIVERS_DIR is the variable ruby-dw1000 reads (ext/vendoring.rb,
# Vendoring.override): it sends this tree to the nodes in place of the
# gem's submodule. The name this line carried until 2026-09-19,
# DW1000_DRIVERS_DIR, was read by nothing, and the runs of 2026-09-18 built
# the submodule's pin, which was this tree's HEAD at the time.
ENV["DECAWAVE_DRIVERS_DIR"] ||= File.expand_path("~/Repos/decawave-drivers")
require "/home/sdalu/Repos/ruby-dw1000/test/support/remote"
jam   = ARGV[0] == "jam"
out   = ARGV[1] || "."
hosts = [ "rpi-c", "rpi-d" ] + (jam ? [ "rpi-b" ] : [])
hosts.each { |h| Remote.prepare(h) }
counter = Remote.role("rpi-c", "count", 60, "stress", env: { "DW1000_DUPLEX_TRACE" => "1" })
counter.await_ready
jammer = jam ? Remote.role("rpi-b", "flood", 3000, "jam", 0).tap(&:await_ready) : nil
flood  = Remote.role("rpi-d", "flood", 3000, "stress", 0)
flood.await_ready
results = [ counter, flood, jammer ].compact.map { |j| j.wait(150) }
puts "== " + (jam ? "with rpi-b jamming" : "baseline")
results.each do |r|
  o = r.output
  File.write(out + "/errbranch-" + (jam ? "jam" : "base") + "-" + r.host + ".log", o)
  passes   = o.lines.grep(/^PASS/)
  last_err = passes.rindex { |l| l =~ /^PASS \S*rx_error/ }
  after    = last_err ? passes[(last_err + 1)..].count { |l| l =~ /^PASS \S*rx_ok/ } : "n/a"
  printf "%s: ok=%s %s discrepancy=%s warnings=%d passes=%d rx_ok=%d rx_error=%d rx_timeout=%d rx_ok_after_last_error=%s\n",
         r.host, r.ok?, r.stats.inspect, o.include?("discrepancy"),
         o.scan(/^(WARN|CORRUPT|dw1000).*/).size, passes.size,
         passes.count { |l| l =~ /^PASS \S*rx_ok/ }, passes.count { |l| l =~ /^PASS \S*rx_error/ },
         passes.count { |l| l =~ /^PASS \S*rx_timeout/ }, after
end
