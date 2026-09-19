# frozen_string_literal: true
# Over a soak's run-*.log traces (DW1000_DUPLEX_TRACE=1): every delivery
# whose entry word has LDEDONE clear, with the buffer pointers, the transmit
# bits, whether its RX_TIME stamp equals an earlier delivery's, and whether
# the next delivery is a duplicate. Usage: analyse-lde.rb DIR
dir = ARGV[0] or abort "usage: analyse-lde.rb DIR"
tot = nolde = stale = dups = 0
Dir["#{dir}/run-*.log"].sort.each do |f|
    rx = File.readlines(f).each_with_index
             .select { |l, _| l.start_with?("RXSEQ") }
             .map { |l, i| s = l.split
                    { ln: i + 1, seq: s[1].to_i, ts: s[3].to_i, w: s[4].to_i(16) } }
    rx.each_with_index do |r, k|
        tot += 1
        w    = r[:w]
        prev = rx[0...k]
        same = prev.find { |q| q[:ts] == r[:ts] }
        stale += 1 if same
        nxt  = rx[k + 1]
        # A redelivery: the next delivery carries a stamp already delivered
        # and no detect bit (a fresh reception sets RXPRD|RXSFDD|RXPHD; a
        # cut frame carries a stale stamp too, but with them set)
        det  = (1 << 8) | (1 << 9) | (1 << 11)
        dup  = nxt && (nxt[:w] & det).zero? && rx[0..k].any? { |q| q[:ts] == nxt[:ts] }
        dups += 1 if dup
        next if w[10] == 1
        nolde += 1
        puts format("%s L%-4d seq %-3d 0x%08x HSRBP %d ICRBP %d tx 0x%x %s%s",
                    File.basename(f), r[:ln], r[:seq], w, w[30], w[31], (w >> 4) & 0xf,
                    same ? "stamp of seq #{same[:seq]} (STALE)" :
                           (r[:ts].zero? ? "stamp 0 (no frame before it)" : "fresh stamp"),
                    dup ? "; next delivery is a DUPLICATE" : "")
    end
end
puts "deliveries #{tot}, LDEDONE clear at entry #{nolde}, stale stamps #{stale}, duplicates #{dups}"
