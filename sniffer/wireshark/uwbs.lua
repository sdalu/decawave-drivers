--
-- Copyright (c) 2026
-- Stephane D'Alu, Inria Chroma / Inria Agora, INSA Lyon, CITI Lab.
--
-- SPDX-License-Identifier: Apache-2.0
--
-- A wireshark dissector for the UWB sniffer's ethernet output.
--
-- The sniffer has two outputs and they are not equally readable. `-w`
-- writes pcapng with link type 195 (IEEE 802.15.4 with FCS), which
-- wireshark already understands completely; nothing here is needed for
-- it. The other output puts each captured frame inside an ethernet
-- frame, behind the 32-byte header sniffer/app/unix/wire.h describes,
-- and to wireshark that is an unknown ethertype carrying opaque bytes.
-- This closes that gap: it reads the header, exposes every field as
-- something you can filter and sort on, and hands the frame itself to
-- wireshark's own 802.15.4 dissector.
--
-- Requires Lua 5.4 or later, which means wireshark 4.4 or later: it uses
-- the bitwise operators, which arrived in Lua 5.3. An older wireshark,
-- built against Lua 5.2, does not merely misread this file, it fails to
-- parse it, and the error it reports names the operator rather than the
-- version. If you see a complaint about ')' expected near '&', that is
-- what it means.
--
-- Installing it: copy into your personal plugin directory, which
-- wireshark names under Help > About > Folders,
--
--     cp uwbs.lua ~/.local/lib/wireshark/plugins/
--
-- then start wireshark, or press Ctrl-Shift-L to reload Lua plugins.
--
--     tcpdump -i eth1 -w - ether proto 0x270f | wireshark -k -i -
--
-- The ethertype it binds to is a preference (Preferences > Protocols >
-- UWBS), defaulting to 9999, which is the sniffer's own default: a
-- capture made with `-P 6666` wants that number here too.
--
-- The offsets below are a transcription of wire.h's table and of
-- nothing else. If the two ever disagree, wire.h is right and this is
-- stale; WIRE_VERSION is what says so, and the version check below
-- reports a header this script was not written for rather than showing
-- plausible nonsense.
--
-- What this does NOT do is read the payload's own protocol. The frame
-- goes to the 802.15.4 dissector, and what is inside that is whatever
-- protocol the network runs; a dissector for it belongs with that
-- protocol's definition, not here, because a copy of a layout kept in
-- a second language is a copy that drifts. The sniffer's own
-- --dissector route exists for exactly that reason.
--

local WIRE_MAGIC        = "UWBS"
local WIRE_VERSION      = 1
local WIRE_HDR_SIZE     = 32
local WIRE_META_SIZE    = 32
local WIRE_POWER_NONE   = -2147483648   -- INT32_MIN

local DEFAULT_ETHERTYPE = 9999          -- 0x270f, the sniffer's default

local p_uwbs = Proto("uwbs", "UWB Sniffer Header")

-- The fixed header.
local f = {
    magic      = ProtoField.string("uwbs.magic",    "Magic"),
    version    = ProtoField.uint8 ("uwbs.version",  "Version", base.DEC),
    hdr_len    = ProtoField.uint8 ("uwbs.hdr_len",  "Header length",
                                   base.DEC),
    flags      = ProtoField.uint16("uwbs.flags",    "Flags", base.HEX),
    truncated  = ProtoField.bool  ("uwbs.truncated",  "Truncated",
                                   16, nil, 0x0001),
    metadata   = ProtoField.bool  ("uwbs.metadata",   "Metadata present",
                                   16, nil, 0x0002),
    ranging    = ProtoField.bool  ("uwbs.ranging",    "Ranging",
                                   16, nil, 0x0004),
    seq        = ProtoField.uint32("uwbs.seq",      "Sequence", base.DEC),
    reported   = ProtoField.uint16("uwbs.reported_len",
                                   "Reported length", base.DEC),
    captured   = ProtoField.uint16("uwbs.captured_len",
                                   "Captured length", base.DEC),
    lost_ring  = ProtoField.uint32("uwbs.lost_ring", "Lost (ring)",
                                   base.DEC),
    lost_chip  = ProtoField.uint32("uwbs.lost_chip", "Lost (chip overrun)",
                                   base.DEC),
    wall       = ProtoField.absolute_time("uwbs.wall", "Captured at",
                                          base.UTC),
    wall_ns    = ProtoField.uint64("uwbs.wall_ns",
                                   "Captured at (ns since the epoch)",
                                   base.DEC),

    -- The optional metadata block.
    rx_time    = ProtoField.uint64("uwbs.rx_time",
                                   "RMARKER (chip ticks)", base.DEC),
    ck_offset  = ProtoField.int32 ("uwbs.clock_offset",
                                   "Clock offset (RX_TTCKO)", base.DEC),
    ck_interval= ProtoField.uint32("uwbs.clock_interval",
                                   "Clock interval (RX_TTCKI)", base.DEC),
    drift      = ProtoField.double("uwbs.clock_drift", "Clock drift"),
    p_signal   = ProtoField.int32 ("uwbs.power_signal",
                                   "Signal power (milli-dBm)", base.DEC),
    p_first    = ProtoField.int32 ("uwbs.power_firstpath",
                                   "First-path power (milli-dBm)",
                                   base.DEC),
    first_path = ProtoField.uint16("uwbs.first_path", "First path index",
                                   base.DEC),
    std_noise  = ProtoField.uint16("uwbs.std_noise", "Noise (std dev)",
                                   base.DEC),
    max_noise  = ProtoField.uint16("uwbs.max_noise", "Noise (LDE threshold)",
                                   base.DEC),
}

p_uwbs.fields = {
    f.magic, f.version, f.hdr_len, f.flags, f.truncated, f.metadata,
    f.ranging, f.seq, f.reported, f.captured, f.lost_ring, f.lost_chip,
    f.wall, f.wall_ns,
    f.rx_time, f.ck_offset, f.ck_interval, f.drift, f.p_signal, f.p_first,
    f.first_path, f.std_noise, f.max_noise,
}

-- Expert info, so that the two things worth noticing about a capture are
-- noticeable rather than buried in a field: a truncated frame is not the
-- frame that was sent, and a loss count that has moved means frames are
-- missing from the capture entirely.
local e_truncated = ProtoExpert.new("uwbs.expert.truncated",
    "Frame was truncated on capture: fewer bytes here than were received",
    expert.group.MALFORMED, expert.severity.WARN)
local e_version = ProtoExpert.new("uwbs.expert.version",
    "Header version this dissector was not written for",
    expert.group.PROTOCOL, expert.severity.WARN)

p_uwbs.experts = { e_truncated, e_version }

p_uwbs.prefs.ethertype = Pref.uint("Ethertype", DEFAULT_ETHERTYPE,
    "The sniffer's --prototype value for this capture (default 9999)")

-- Wireshark's own 802.15.4 dissector. The frame the sniffer forwards
-- includes the CRC, which is why this is "wpan" and not "wpan_nofcs",
-- and why the pcapng path uses link type 195 rather than 230.
local wpan = Dissector.get("wpan")

local function power_text(milli)
    if milli == WIRE_POWER_NONE then
        return "n/a"
    end
    return string.format("%.3f dBm", milli / 1000.0)
end

function p_uwbs.dissector(tvb, pinfo, tree)
    local len = tvb:len()

    if len < WIRE_HDR_SIZE then
        return 0
    end

    -- The magic is what tells this stream from the bare-frame one the
    -- sniffer's --raw still produces. Returning 0 rather than flagging an
    -- error is deliberate: with --raw these bytes are simply somebody
    -- else's frame, and wireshark should go on looking.
    if tvb(0, 4):string() ~= WIRE_MAGIC then
        return 0
    end

    local version = tvb(4, 1):uint()
    local hdr_len = tvb(5, 1):uint()

    if hdr_len < WIRE_HDR_SIZE or hdr_len > len then
        return 0
    end

    local flags    = tvb(6, 2):le_uint()
    local seq      = tvb(8, 4):le_uint()
    local reported = tvb(12, 2):le_uint()
    local captured = tvb(14, 2):le_uint()
    local has_meta = (flags & 0x0002) ~= 0

    local t = tree:add(p_uwbs, tvb(0, hdr_len),
                       string.format("UWB Sniffer Header, seq %d, %d bytes",
                                     seq, captured))

    t:add(f.magic,   tvb(0, 4))
    t:add(f.version, tvb(4, 1))
    t:add(f.hdr_len, tvb(5, 1))

    if version ~= WIRE_VERSION then
        t:add_proto_expert_info(e_version,
            string.format("header says version %d, this dissector"
                          .. " reads version %d", version, WIRE_VERSION))
    end

    local ft = t:add_le(f.flags, tvb(6, 2))
    ft:add_le(f.truncated, tvb(6, 2))
    ft:add_le(f.metadata,  tvb(6, 2))
    ft:add_le(f.ranging,   tvb(6, 2))

    t:add_le(f.seq,       tvb(8, 4))
    t:add_le(f.reported,  tvb(12, 2))
    t:add_le(f.captured,  tvb(14, 2))
    t:add_le(f.lost_ring, tvb(16, 4))
    t:add_le(f.lost_chip, tvb(20, 4))

    -- One 64-bit count of nanoseconds since the epoch, shown both as a
    -- time and as the raw number: the time is what you read, the number
    -- is what you filter on without worrying about formatting.
    local ns  = tvb(24, 8):le_uint64()
    local sec = (ns / 1000000000):tonumber()
    local rem = (ns % 1000000000):tonumber()
    t:add(f.wall, tvb(24, 8), NSTime.new(sec, rem))
    t:add_le(f.wall_ns, tvb(24, 8))

    if (flags & 0x0001) ~= 0 then
        t:add_proto_expert_info(e_truncated,
            string.format("%d bytes captured of %d received",
                          captured, reported))
    end

    if has_meta and hdr_len >= (WIRE_HDR_SIZE + WIRE_META_SIZE) then
        local m = t:add(tvb(WIRE_HDR_SIZE, WIRE_META_SIZE), "Radio metadata")
        local o = WIRE_HDR_SIZE

        m:add_le(f.rx_time, tvb(o, 8))

        local offset   = tvb(o + 8, 4):le_int()
        local interval = tvb(o + 12, 4):le_uint()
        m:add_le(f.ck_offset,   tvb(o + 8, 4))
        m:add_le(f.ck_interval, tvb(o + 12, 4))
        -- interval 0 means RX_TTCKI had not been written yet, so there is
        -- no drift to compute rather than a drift of zero. Said, not
        -- divided by.
        if interval ~= 0 then
            m:add(f.drift, tvb(o + 8, 8), offset / interval)
        else
            m:add(tvb(o + 12, 4), "Clock drift: n/a (interval is 0)")
        end

        local psig = tvb(o + 16, 4):le_int()
        local pfp  = tvb(o + 20, 4):le_int()
        m:add_le(f.p_signal, tvb(o + 16, 4))
            :append_text(" (" .. power_text(psig) .. ")")
        m:add_le(f.p_first, tvb(o + 20, 4))
            :append_text(" (" .. power_text(pfp) .. ")")

        m:add_le(f.first_path, tvb(o + 24, 2))
        m:add_le(f.std_noise,  tvb(o + 26, 2))
        m:add_le(f.max_noise,  tvb(o + 28, 2))

        t:append_text(string.format(", signal %s", power_text(psig)))
    end

    pinfo.cols.protocol = "UWBS"
    pinfo.cols.info = string.format("seq %d, %d bytes%s", seq, captured,
                                    ((flags & 0x0001) ~= 0)
                                        and " (truncated)" or "")

    -- The captured frame itself. captured_len and not the rest of the
    -- ethernet payload: a frame shorter than 46 bytes was padded to
    -- ETH_ZLEN by the sending NIC, and handing that padding to the
    -- 802.15.4 dissector is exactly the confusion this header exists to
    -- prevent.
    local avail = len - hdr_len
    local n     = (captured < avail) and captured or avail

    if n > 0 then
        wpan:call(tvb(hdr_len, n):tvb(), pinfo, tree)
    end

    return hdr_len + n
end

local function register()
    DissectorTable.get("ethertype"):add(p_uwbs.prefs.ethertype, p_uwbs)
end

function p_uwbs.prefs_changed()
    -- Wireshark keeps the old binding until it is reloaded, so say what
    -- to do rather than leaving a changed preference looking broken.
    register()
end

register()
