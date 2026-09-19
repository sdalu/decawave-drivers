# frozen_string_literal: true
# INVESTIGATE.md entry 1: the duplex run, the receiver-enable placement
# alternated run by run (even runs: end of the pass, the driver's
# default; odd runs: inside the pass, DW1000_RX_ENABLE_EARLY=1), every
# run's output kept, one JSON line per node per run.
# DECAWAVE_DRIVERS_DIR is the variable ruby-dw1000 reads (ext/vendoring.rb,
# Vendoring.override): it sends this tree to the nodes in place of the
# gem's submodule. The name this line carried until 2026-09-19,
# DW1000_DRIVERS_DIR, was read by nothing, and the runs of 2026-09-18 built
# the submodule's pin, which was this tree's HEAD at the time.
ENV["DECAWAVE_DRIVERS_DIR"] ||= File.expand_path("~/Repos/decawave-drivers")
require "/home/sdalu/Repos/ruby-dw1000/test/support/remote"
require "json"

runs  = Integer(ARGV[0] || 48)
out   = ARGV[1] or abort "usage: soak.rb RUNS OUTDIR [FIRST]"
first = Integer(ARGV[2] || 0)
count = 50
hosts = [ Remote::HOST_A, Remote::HOST_B ]
hosts.each { |h| Remote.prepare(h) }

(first...(first + runs)).each do |i|
    early = false
    env   = { "DW1000_DUPLEX_TRACE" => "1" }
    env["DW1000_RX_ENABLE_EARLY"] = "1" if early
    t0    = Time.now
    begin
        jobs = [ [ Remote::HOST_A, "dup-a", "dup-b" ],
                 [ Remote::HOST_B, "dup-b", "dup-a" ] ].map do |h, me, peer|
            Remote.role(h, "duplex", count, me, peer, env: env)
        end
        jobs.each(&:await_ready)
        jobs.each(&:go) if ENV["SOAK_GO"] == "1"
        results = jobs.map { |j| j.wait(150) }
    rescue Remote::Error => e
        rec = { run: i, placement: early ? "early" : "end", error: e.message[0, 300] }
        puts rec.to_json
        File.open("#{out}/soak.jsonl", "a") { |f| f.puts rec.to_json }
        jobs&.each { |j| j.stop rescue nil }
        next
    end
    results.each do |r|
        File.write("#{out}/run-#{format('%03d', i)}-#{r.host}.log", r.output)
        rec = { run: i, placement: early ? "early" : "end", host: r.host,
                ok: r.ok?, secs: (Time.now - t0).round(1) }
        rec.merge!(r.stats.transform_keys(&:to_sym)) rescue rec[:nostats] = true
        rec[:discrepancy] = true if r.output.include?("discrepancy")
        puts rec.to_json
        File.open("#{out}/soak.jsonl", "a") { |f| f.puts rec.to_json }
    end
end
