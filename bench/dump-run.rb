# frozen_string_literal: true
# dump.rb DIR RUN HOST LO HI: the log lines between seq LO and HI, pass words decoded
NAMES = { 1=>"CPLOCK",3=>"AAT",4=>"TXFRB",5=>"TXPRS",6=>"TXPHS",7=>"TXFRS",8=>"RXPRD",9=>"RXSFDD",10=>"LDEDONE",11=>"RXPHD",12=>"RXPHE",13=>"RXDFR",14=>"RXFCG",15=>"RXFCE",16=>"RXRFSL",17=>"RXRFTO",18=>"LDEERR",20=>"RXOVRR",21=>"RXPTO",23=>"SLP2INIT",26=>"RXSFDTO",27=>"HPDWARN",28=>"TXBERR",29=>"AFFREJ",30=>"HSRBP",31=>"ICRBP" }
def dec(w) = NAMES.select { |b, _| w[b] == 1 }.values.join("|")
dir, run, host, lo, hi = ARGV[0], ARGV[1].to_i, ARGV[2], ARGV[3].to_i, ARGV[4].to_i
lines = File.readlines("#{dir}/run-#{format('%03d', run)}-#{host}.log")
from = lines.index { |l| l =~ /^(RXSEQ|TXSEQ) (\d+)/ && $2.to_i >= lo } || 0
to   = lines.rindex { |l| l =~ /^(RXSEQ|TXSEQ) (\d+)/ && $2.to_i <= hi } || lines.size - 1
puts "== run #{run} #{host} (#{run.odd? ? 'early' : 'end'}) seqs #{lo}..#{hi}"
lines[from..to].each do |l|
  a = l.split
  case a[0]
  when "PASS"
    ev, *w = a[1..]
    if ev.start_with?("0x") then w.unshift(ev); ev = "(none)" end
    w = w.map { _1.to_i(16) }
    puts format("  PASS %-14s %s", ev, w.map { |x| dec(x) }.join("  /  "))
  when "RXSEQ" then puts format("  RXSEQ %-3s t=%s ts=%s %s", a[1], a[2], a[3], dec(a[4].to_i(16)))
  when "TXSEQ" then puts "  TXSEQ #{a[1]} ts=#{a[2]}"
  else puts "  #{l.strip}"
  end
end
