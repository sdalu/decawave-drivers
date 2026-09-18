--
-- Copyright (c) 2026
-- Stephane D'Alu, Inria Chroma / Inria Agora, INSA Lyon, CITI Lab.
--
-- SPDX-License-Identifier: Apache-2.0
--
-- Runs sniffer/wireshark/uwbs.lua outside wireshark and checks what it
-- makes of a header the real wire.c encoded.
--
-- The expected values are not written here. tests/lua/emit.c encodes a
-- header with wire_encode() and prints both the bytes and the values it
-- put in them; this reads that, hands the bytes to the dissector, and
-- compares field by field. So the comparison is the C encoder against
-- the Lua decoder, with nothing transcribed from wire.h twice, which is
-- the mistake a hand-typed header would have made: the test would have
-- agreed with itself.
--
-- What is checked: the magic and version, every field of the fixed
-- header at whatever offset the dissector chose to read it from, the
-- metadata block when present and its absence when not, the
-- little-endian decoding (the stub honours wireshark's add versus add_le
-- distinction, so a field read big endian fails here), the truncation
-- expert info, how much of the payload reaches the 802.15.4 dissector,
-- the registration on the ethertype, and the two cases the dissector
-- must decline: a buffer too short and a payload whose magic is not
-- ours (which is what --raw produces).
--
-- What is NOT checked is whether wireshark agrees with the stub about
-- the API. tests/lua/wsstub.lua says so at length.
--
-- usage: lua uwbs_test.lua <dissector.lua> <emit-binary>

local failures = 0

local function step(what, reason)
    if reason == nil then
        print("ok: " .. what)
    else
        print("FAIL: " .. what .. " (" .. reason .. ")")
        failures = failures + 1
    end
end

local dissector_path = arg[1]
local emit_path      = arg[2]

if (dissector_path == nil) or (emit_path == nil) then
    io.stderr:write("usage: lua uwbs_test.lua <dissector.lua> <emit>\n")
    os.exit(2)
end

-- The stub sits beside this file.
local here = arg[0]:match("^(.*)/[^/]*$") or "."
package.path = here .. "/?.lua;" .. package.path
local ws = require("wsstub")

--=======================================================================
-- Reading emit.c's output
--=======================================================================

local function emit(case)
    local h = io.popen(string.format("%s %s", emit_path, case), "r")
    if h == nil then
        error("cannot run " .. emit_path)
    end
    local out = h:read("*a")
    h:close()

    local t = {}
    for line in out:gmatch("[^\n]+") do
        local k, v = line:match("^([%w_]+)=(.*)$")
        if k ~= nil then
            t[k] = v
        end
    end
    if t.hex == nil then
        error("no hex= in the output of `" .. emit_path .. " " .. case .. "`")
    end
    return t
end

local function unhex(s)
    return (s:gsub("%x%x", function (b)
        return string.char(tonumber(b, 16))
    end))
end

local function num(t, k)
    local v = tonumber(t[k])
    if v == nil then
        error("emit did not print " .. k)
    end
    return v
end

--=======================================================================
-- Loading the dissector, once, with the stub installed
--=======================================================================

ws.install()
ws.reset()

local chunk, err = loadfile(dissector_path)
if chunk == nil then
    step("the dissector loads", err)
    os.exit(1)
end

local ok, lerr = pcall(chunk)
if not ok then
    step("the dissector loads", tostring(lerr))
    os.exit(1)
end

local proto = ws.protos["uwbs"]
if proto == nil then
    step("the dissector loads", "it declared no Proto named 'uwbs'")
    os.exit(1)
end
step("the dissector loads and declares Proto 'uwbs'", nil)

--=======================================================================
-- Registration
--=======================================================================

local function case_registration()
    if #ws.registrations == 0 then
        return "it registered on no dissector table"
    end
    local r = ws.registrations[1]
    if r.table ~= "ethertype" then
        return "registered on table '" .. tostring(r.table) .. "'"
    end
    -- 9999 is --prototype's default, and wire.h explains why the default
    -- is not something smaller.
    if r.pattern ~= 9999 then
        return string.format("registered on ethertype %s, want 9999",
                             tostring(r.pattern))
    end
    if r.proto ~= proto then
        return "registered something other than the uwbs proto"
    end
    return nil
end

--=======================================================================
-- Driving one case
--=======================================================================

-- Run the dissector over a header plus its frame, and hand back the
-- tree, the consumed count, and the stub's record of what it called.
local function run(t)
    local hdr   = unhex(t.hex)
    local frame = unhex(t.framehex)
    local tvb   = ws.Tvb(hdr .. frame)
    local tree  = ws.tree()
    local pinfo = { cols = {} }

    ws.reset()
    local consumed = proto.dissector(tvb, pinfo, tree)
    return tree, consumed, pinfo
end

local function want(tree, abbr, expected, label)
    local e = tree.by_abbr[abbr]
    if e == nil then
        return string.format("%s: the dissector never added %s", label, abbr)
    end
    if e.value ~= expected then
        return string.format("%s: %s is %s, want %s", label, abbr,
                             tostring(e.value), tostring(expected))
    end
    return nil
end

--=======================================================================
-- The fixed header, against what emit.c encoded
--=======================================================================

local function case_fixed_header()
    local t = emit("meta")
    local tree, consumed = run(t)
    local e

    e = want(tree, "uwbs.magic", "UWBS", "meta");             if e then return e end
    e = want(tree, "uwbs.version", num(t, "version"), "meta");  if e then return e end
    e = want(tree, "uwbs.hdr_len", num(t, "hdr_len"), "meta");  if e then return e end
    e = want(tree, "uwbs.seq", num(t, "seq"), "meta");          if e then return e end
    e = want(tree, "uwbs.reported_len", num(t, "reported"), "meta")
    if e then return e end
    e = want(tree, "uwbs.captured_len", num(t, "captured"), "meta")
    if e then return e end
    e = want(tree, "uwbs.lost_ring", num(t, "lost_ring"), "meta")
    if e then return e end
    e = want(tree, "uwbs.lost_chip", num(t, "lost_chip"), "meta")
    if e then return e end

    -- The wall clock: one 64-bit nanosecond count, which the dissector
    -- also splits into an NSTime. Both are checked, because the split is
    -- where a division mistake would hide.
    local want_ns = num(t, "wall_sec") * 1000000000 + num(t, "wall_nsec")
    local ns = tree.by_abbr["uwbs.wall_ns"]
    if ns == nil then
        return "meta: no uwbs.wall_ns"
    end
    if ns.value ~= want_ns then
        return string.format("meta: wall_ns is %s, want %d",
                             tostring(ns.value), want_ns)
    end

    local wall = tree.by_abbr["uwbs.wall"]
    if wall == nil then
        return "meta: no uwbs.wall"
    end
    if type(wall.value) ~= "table" then
        return "meta: uwbs.wall was not given an NSTime"
    end
    if wall.value.sec ~= num(t, "wall_sec") then
        return string.format("meta: wall seconds %s, want %d",
                             tostring(wall.value.sec), num(t, "wall_sec"))
    end
    if wall.value.nsec ~= num(t, "wall_nsec") then
        return string.format("meta: wall nanoseconds %s, want %d",
                             tostring(wall.value.nsec), num(t, "wall_nsec"))
    end

    -- The flags, as booleans off the same two bytes.
    e = want(tree, "uwbs.truncated", false, "meta");  if e then return e end
    e = want(tree, "uwbs.metadata", true, "meta");    if e then return e end
    e = want(tree, "uwbs.ranging", true, "meta");     if e then return e end

    -- Everything consumed: the header plus the frame, and no padding.
    local want_consumed = num(t, "hdr_len") + num(t, "captured")
    if consumed ~= want_consumed then
        return string.format("meta: consumed %s, want %d",
                             tostring(consumed), want_consumed)
    end

    return nil
end

--=======================================================================
-- The metadata block
--=======================================================================

local function case_metadata()
    local t = emit("meta")
    local tree = run(t)
    local e

    e = want(tree, "uwbs.rx_time", num(t, "rx_time"), "meta")
    if e then return e end
    e = want(tree, "uwbs.clock_offset", num(t, "clock_offset"), "meta")
    if e then return e end
    e = want(tree, "uwbs.clock_interval", num(t, "clock_interval"), "meta")
    if e then return e end
    e = want(tree, "uwbs.power_signal", num(t, "power_signal"), "meta")
    if e then return e end
    e = want(tree, "uwbs.power_firstpath", num(t, "power_firstpath"), "meta")
    if e then return e end
    e = want(tree, "uwbs.first_path", num(t, "first_path"), "meta")
    if e then return e end
    e = want(tree, "uwbs.std_noise", num(t, "std_noise"), "meta")
    if e then return e end
    e = want(tree, "uwbs.max_noise", num(t, "max_noise"), "meta")
    if e then return e end

    -- A negative clock offset has to survive its unsigned encoding, and
    -- emit.c deliberately uses one.
    if num(t, "clock_offset") >= 0 then
        return "the fixture's clock_offset is not negative; it should be"
    end

    -- The drift the dissector computes from the pair.
    local drift = tree.by_abbr["uwbs.clock_drift"]
    if drift == nil then
        return "no uwbs.clock_drift with a non-zero interval"
    end
    local want_drift = num(t, "clock_offset") / num(t, "clock_interval")
    if math.abs(drift.value - want_drift) > 1e-12 then
        return string.format("drift %s, want %s",
                             tostring(drift.value), tostring(want_drift))
    end

    -- emit.c sets one power to the sentinel, so the dissector's "n/a"
    -- path is exercised: the field still carries the raw value, and the
    -- text beside it must not read as a number of dBm.
    local fp = tree.by_abbr["uwbs.power_firstpath"]
    if (fp.text == nil) or (not fp.text:find("n/a", 1, true)) then
        return string.format("firstpath text is %q, want it to say n/a",
                             tostring(fp.text))
    end
    local sig = tree.by_abbr["uwbs.power_signal"]
    if (sig.text == nil) or (not sig.text:find("dBm", 1, true)) then
        return string.format("signal text is %q, want dBm",
                             tostring(sig.text))
    end

    return nil
end

-- The metadata bit and the ranging bit, told apart.
--
-- Every other fixture has both set, so a dissector that read the ranging
-- bit where it meant the metadata bit would behave identically on all of
-- them. This one has metadata set and ranging clear: get the bits the
-- wrong way round and the block is not read at all.
local function case_metadata_without_ranging()
    local t = emit("metaonly")
    local tree = run(t)

    if num(t, "has_meta") ~= 1 then
        return "the metaonly fixture carries no metadata"
    end
    if num(t, "ranging") ~= 0 then
        return "the metaonly fixture has the ranging bit set; it should not"
    end

    local e = want(tree, "uwbs.metadata", true, "metaonly")
    if e then return e end
    e = want(tree, "uwbs.ranging", false, "metaonly")
    if e then return e end

    -- The block itself must have been read, which is what fails when the
    -- two bits are confused.
    e = want(tree, "uwbs.rx_time", num(t, "rx_time"), "metaonly")
    if e then return e end
    e = want(tree, "uwbs.max_noise", num(t, "max_noise"), "metaonly")
    if e then return e end

    return nil
end

local function case_no_metadata()
    local t = emit("nometa")
    local tree, consumed = run(t)

    if num(t, "has_meta") ~= 0 then
        return "the nometa fixture carries metadata"
    end
    if num(t, "hdr_len") ~= 32 then
        return string.format("the nometa header is %d bytes, want 32",
                             num(t, "hdr_len"))
    end

    local e = want(tree, "uwbs.metadata", false, "nometa")
    if e then return e end

    -- Nothing from the block may appear: reading it out of a header that
    -- does not have it is exactly what hdr_len exists to prevent.
    for _, abbr in ipairs({ "uwbs.rx_time", "uwbs.clock_offset",
                            "uwbs.clock_interval", "uwbs.power_signal",
                            "uwbs.power_firstpath", "uwbs.first_path",
                            "uwbs.std_noise", "uwbs.max_noise" }) do
        if tree.by_abbr[abbr] ~= nil then
            return "nometa: " .. abbr .. " was read from a header without it"
        end
    end

    local want_consumed = 32 + num(t, "captured")
    if consumed ~= want_consumed then
        return string.format("nometa: consumed %s, want %d",
                             tostring(consumed), want_consumed)
    end

    return nil
end

--=======================================================================
-- Truncation, and the payload hand-off
--=======================================================================

local function case_truncated()
    local t = emit("trunc")
    local tree = run(t)

    if num(t, "truncated") ~= 1 then
        return "the trunc fixture is not flagged truncated"
    end
    if num(t, "reported") <= num(t, "captured") then
        return "the trunc fixture's reported length is not the larger"
    end

    local e = want(tree, "uwbs.truncated", true, "trunc")
    if e then return e end
    e = want(tree, "uwbs.reported_len", num(t, "reported"), "trunc")
    if e then return e end
    e = want(tree, "uwbs.captured_len", num(t, "captured"), "trunc")
    if e then return e end

    -- Truncation is worth an expert note rather than only a field.
    local found = false
    for _, x in ipairs(tree.experts) do
        if x.expert.abbr == "uwbs.expert.truncated" then
            found = true
        end
    end
    if not found then
        return "a truncated frame raised no expert info"
    end

    return nil
end

local function case_payload()
    local t = emit("meta")
    local tree = run(t)
    local frame = unhex(t.framehex)

    if #ws.calls ~= 1 then
        return string.format("%d dissectors called, want 1", #ws.calls)
    end
    local c = ws.calls[1]
    if c.name ~= "wpan" then
        return "handed the payload to '" .. tostring(c.name) ..
               "', want 'wpan' (802.15.4 with FCS)"
    end
    if c.len ~= num(t, "captured") then
        return string.format("handed on %d bytes, want %d (captured_len)",
                             c.len, num(t, "captured"))
    end
    if c.bytes ~= frame then
        return "the bytes handed on are not the captured frame"
    end
    return nil
end

-- A frame padded by the sending NIC: captured_len says 13, the ethernet
-- payload is 60. Only the frame may reach the 802.15.4 dissector, which
-- is the whole reason captured_len is on the wire.
local function case_padding()
    local t     = emit("meta")
    local hdr   = unhex(t.hex)
    local frame = unhex(t.framehex)
    local pad   = string.rep("\0", 60 - #frame)
    local tvb   = ws.Tvb(hdr .. frame .. pad)
    local tree  = ws.tree()

    ws.reset()
    local consumed = proto.dissector(tvb, { cols = {} }, tree)

    if #ws.calls ~= 1 then
        return "the padded frame did not reach one dissector"
    end
    if ws.calls[1].len ~= num(t, "captured") then
        return string.format("handed on %d bytes of a padded frame, want %d",
                             ws.calls[1].len, num(t, "captured"))
    end
    local want_consumed = num(t, "hdr_len") + num(t, "captured")
    if consumed ~= want_consumed then
        return string.format("consumed %s of a padded frame, want %d",
                             tostring(consumed), want_consumed)
    end
    return nil
end

--=======================================================================
-- What it must decline
--=======================================================================

local function case_declines()
    -- Too short to hold a header.
    local tree = ws.tree()
    ws.reset()
    if proto.dissector(ws.Tvb(string.rep("x", 8)), { cols = {} }, tree) ~= 0 then
        return "a buffer too short for a header was not declined"
    end
    if #ws.calls ~= 0 then
        return "a short buffer still reached another dissector"
    end

    -- The magic is wrong: this is what --raw produces, and wireshark
    -- should go on looking rather than be told it is ours.
    local t    = emit("meta")
    local hdr  = unhex(t.hex)
    local raw  = "XXXX" .. hdr:sub(5)
    tree = ws.tree()
    ws.reset()
    if proto.dissector(ws.Tvb(raw .. unhex(t.framehex)), { cols = {} },
                       tree) ~= 0 then
        return "a payload with the wrong magic was claimed anyway"
    end
    if #tree.entries ~= 0 then
        return "a payload with the wrong magic was added to the tree"
    end

    -- A header claiming to be shorter than the fixed part, or longer
    -- than the buffer, is not something to walk.
    local bad = hdr:sub(1, 5) .. string.char(8) .. hdr:sub(7)
    tree = ws.tree()
    ws.reset()
    if proto.dissector(ws.Tvb(bad .. unhex(t.framehex)), { cols = {} },
                       tree) ~= 0 then
        return "a header claiming hdr_len 8 was walked anyway"
    end

    local long = hdr:sub(1, 5) .. string.char(200) .. hdr:sub(7)
    tree = ws.tree()
    ws.reset()
    if proto.dissector(ws.Tvb(long), { cols = {} }, tree) ~= 0 then
        return "a header longer than the buffer was walked anyway"
    end

    return nil
end

-- A version the dissector was not written for is reported, not guessed
-- at: the fields are still shown, with an expert note beside them.
local function case_version()
    local t    = emit("meta")
    local hdr  = unhex(t.hex)
    local next_version = string.char((num(t, "version") + 1) % 256)
    local bumped = hdr:sub(1, 4) .. next_version .. hdr:sub(6)
    local tree = ws.tree()

    ws.reset()
    local consumed = proto.dissector(ws.Tvb(bumped .. unhex(t.framehex)),
                                     { cols = {} }, tree)
    if consumed == 0 then
        return "a future version was declined rather than reported"
    end

    local found = false
    for _, x in ipairs(tree.experts) do
        if x.expert.abbr == "uwbs.expert.version" then
            found = true
        end
    end
    if not found then
        return "a future version raised no expert info"
    end
    return nil
end

--=======================================================================

step("registers on ethertype 9999",            case_registration())
step("the fixed header, field by field",       case_fixed_header())
step("the metadata block, and its sentinels",  case_metadata())
step("metadata set, ranging clear: the bits apart",
					     case_metadata_without_ranging())
step("a header with no metadata block",        case_no_metadata())
step("a truncated frame, flag and expert",     case_truncated())
step("the frame reaches the 802.15.4 dissector", case_payload())
step("ethernet padding is not handed on",      case_padding())
step("declines what is not its own",           case_declines())
step("a future header version is reported",    case_version())

os.exit(failures == 0 and 0 or 1)
