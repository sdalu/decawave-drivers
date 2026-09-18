--
-- Copyright (c) 2026
-- Stephane D'Alu, Inria Chroma / Inria Agora, INSA Lyon, CITI Lab.
--
-- SPDX-License-Identifier: Apache-2.0
--
-- Enough of wireshark's Lua API to load and run a dissector outside
-- wireshark, for tests/lua/uwbs_test.lua.
--
-- What this buys, and what it does not. It buys running
-- sniffer/wireshark/uwbs.lua at all: its offsets, its endianness, its
-- arithmetic, its magic and version checks, how much of the payload it
-- hands on. Those are the things most likely to be wrong and the things
-- tests/check-wireshark.sh can only partly see. It does NOT buy any
-- assurance that the API is used the way wireshark expects: this file is
-- an imitation written from the same documentation the dissector was,
-- so a misunderstanding shared by both passes here and fails there.
-- Testing against a mock proves the mock agrees. sniffer/DESIGN.md says
-- so, and the dissector still wants opening in wireshark once by hand.
--
-- The one place that distinction is taken seriously is endianness.
-- wireshark's tree:add() reads a field big endian and tree:add_le()
-- little endian, and getting that backwards on a little-endian wire
-- format is a real and silent bug. So the decoding below honours the
-- difference rather than ignoring it, and the test checks values, not
-- merely that add was called.
--
-- Needs Lua 5.4, the same minimum the dissector has. Two things want
-- it: 64-bit integers, because the wall clock is nanoseconds since the
-- epoch (about 1.7e18, which a double cannot hold exactly), and the
-- bitwise operators the decoding below uses.
--

local M = {}

--=======================================================================
-- Values a dissector reads out of a Tvb
--=======================================================================

-- wireshark hands back a UInt64 object, not a plain number, and a
-- dissector does arithmetic on it. Division has to truncate the way
-- wireshark's does; Lua's / would make it a float and lose the low
-- digits of a nanosecond count.
local UInt64 = {}
UInt64.__index = UInt64

local function uint64(v)
    return setmetatable({ v = v }, UInt64)
end

function UInt64:tonumber() return self.v end
function UInt64:__tostring() return string.format("%d", self.v) end

UInt64.__div = function (a, b)
    local av = (type(a) == "table") and a.v or a
    local bv = (type(b) == "table") and b.v or b
    return uint64(av // bv)
end

UInt64.__mod = function (a, b)
    local av = (type(a) == "table") and a.v or a
    local bv = (type(b) == "table") and b.v or b
    return uint64(av % bv)
end

UInt64.__eq = function (a, b) return a.v == b.v end

M.uint64 = uint64

--=======================================================================
-- Tvb and TvbRange
--=======================================================================

local TvbRange = {}
TvbRange.__index = TvbRange

local function decode_uint(bytes, le)
    local v = 0
    if le then
        for i = #bytes, 1, -1 do
            v = v * 256 + bytes:byte(i)
        end
    else
        for i = 1, #bytes do
            v = v * 256 + bytes:byte(i)
        end
    end
    return v
end

local function decode_int(bytes, le)
    local v    = decode_uint(bytes, le)
    local bits = #bytes * 8
    local half = 1 << (bits - 1)
    if v >= half then
        v = v - (half + half)
    end
    return v
end

function TvbRange:len()        return #self.bytes end
function TvbRange:string()     return self.bytes end
function TvbRange:bytes()      return self.bytes end
function TvbRange:uint()       return decode_uint(self.bytes, false) end
function TvbRange:le_uint()    return decode_uint(self.bytes, true) end
function TvbRange:int()        return decode_int(self.bytes, false) end
function TvbRange:le_int()     return decode_int(self.bytes, true) end
function TvbRange:uint64()     return uint64(decode_uint(self.bytes, false)) end
function TvbRange:le_uint64()  return uint64(decode_uint(self.bytes, true)) end
function TvbRange:offset()     return self.off end

-- A range promoted back to a Tvb, which is how a dissector hands a
-- payload to another dissector.
function TvbRange:tvb()        return M.Tvb(self.bytes) end

local Tvb = {}

-- Callable, because a dissector writes tvb(offset, length). An offset or
-- length outside the buffer is an error in wireshark, and raising here
-- rather than clamping is the point: a dissector that walks off the end
-- should fail the test, not quietly read short.
local function tvb_call(self, off, len)
    if off == nil then
        return setmetatable({ bytes = self.buf, off = 0 }, TvbRange)
    end
    if len == nil then
        len = #self.buf - off
    end
    if (off < 0) or (len < 0) or ((off + len) > #self.buf) then
        error(string.format(
            "tvb(%d, %d) is outside a %d byte buffer", off, len, #self.buf), 2)
    end
    return setmetatable({ bytes = self.buf:sub(off + 1, off + len), off = off },
                        TvbRange)
end

function M.Tvb(buf)
    local t = setmetatable({ buf = buf }, {
        __index = Tvb,
        __call  = tvb_call,
    })
    return t
end

function Tvb:len()      return #self.buf end
function Tvb:bytes()    return self.buf end

--=======================================================================
-- Fields
--=======================================================================

local function field(kind, width, abbr, name, extra)
    return { kind = kind, width = width, abbr = abbr, name = name,
             extra = extra }
end

-- wireshark's constructors are (abbr, name, base, valuestring, mask,
-- desc); the trailing ones are accepted and ignored here, except bool's
-- width and mask, which the decoding below needs.
M.ProtoField = {
    string        = function (a, n, ...)  return field("string", nil, a, n) end,
    uint8         = function (a, n, ...)  return field("uint", 1, a, n)     end,
    uint16        = function (a, n, ...)  return field("uint", 2, a, n)     end,
    uint32        = function (a, n, ...)  return field("uint", 4, a, n)     end,
    uint64        = function (a, n, ...)  return field("uint", 8, a, n)     end,
    int32         = function (a, n, ...)  return field("int", 4, a, n)      end,
    double        = function (a, n, ...)  return field("double", nil, a, n) end,
    absolute_time = function (a, n, ...)  return field("time", nil, a, n)   end,
    bool          = function (a, n, w, _, mask)
                        return field("bool", (w or 8) / 8, a, n, mask)
                    end,
    bytes         = function (a, n, ...)  return field("bytes", nil, a, n)  end,
}

M.base   = { DEC = "DEC", HEX = "HEX", OCT = "OCT", UTC = "UTC",
             NONE = "NONE", UNIT_STRING = "UNIT_STRING" }
M.expert = {
    group    = { MALFORMED = "MALFORMED", PROTOCOL = "PROTOCOL",
                 CHECKSUM = "CHECKSUM", UNDECODED = "UNDECODED" },
    severity = { WARN = "WARN", NOTE = "NOTE", ERROR = "ERROR",
                 CHAT = "CHAT" },
}

M.ProtoExpert = {
    new = function (abbr, text, group, severity)
        return { abbr = abbr, text = text, group = group,
                 severity = severity }
    end,
}

-- Reading p.prefs.<name> in wireshark gives the preference's value, so
-- the stub's constructor simply is the default value.
M.Pref = {
    uint   = function (_, default) return default end,
    bool   = function (_, default) return default end,
    string = function (_, default) return default end,
    enum   = function (_, default) return default end,
}

M.NSTime = {
    new = function (sec, nsec) return { sec = sec, nsec = nsec } end,
}

--=======================================================================
-- The tree
--=======================================================================

local TreeItem = {}
TreeItem.__index = TreeItem

-- Every add() lands here, recorded flat on the root so that a test can
-- look a field up by its abbreviation without walking the tree.
local function record(root, entry)
    table.insert(root.entries, entry)
    if entry.abbr ~= nil then
        root.by_abbr[entry.abbr] = entry
    end
end

local function decode_for(f, range, le)
    if f.kind == "string" then
        return range:string()
    elseif f.kind == "uint" then
        return le and range:le_uint() or range:uint()
    elseif f.kind == "int" then
        return le and range:le_int() or range:int()
    elseif f.kind == "bool" then
        local v = le and range:le_uint() or range:uint()
        return (v % ((f.extra or 1) * 2)) >= (f.extra or 1)
    else
        -- double, time, bytes: wireshark decodes these too, but the
        -- dissector under test always supplies them as an explicit
        -- value, so the stub does not guess.
        return nil
    end
end

local function add_common(self, le, a, b, c)
    local root = self.root or self
    local entry

    if type(a) == "table" and a.kind ~= nil then
        -- add(field, range [, value])
        local value = c
        if value == nil then
            value = decode_for(a, b, le)
        end
        entry = { abbr = a.abbr, name = a.name, field = a, range = b,
                  value = value, le = le, text = nil }
    elseif type(a) == "table" and a.is_proto then
        -- add(proto, range, text)
        entry = { abbr = a.name_abbr, name = a.description, proto = a,
                  range = b, text = c }
    else
        -- add(range, text)
        entry = { range = a, text = b }
    end

    record(root, entry)

    local item = setmetatable({ root = root, entry = entry }, TreeItem)
    return item
end

function TreeItem:add(a, b, c)    return add_common(self, false, a, b, c) end
function TreeItem:add_le(a, b, c) return add_common(self, true,  a, b, c) end

function TreeItem:append_text(s)
    self.entry.text = (self.entry.text or "") .. s
    return self
end

function TreeItem:add_proto_expert_info(e, text)
    local root = self.root or self
    table.insert(root.experts, { expert = e, text = text })
    return self
end

function TreeItem:set_generated() return self end
function TreeItem:set_text(s) self.entry.text = s; return self end

function M.tree()
    local t = setmetatable({}, TreeItem)
    t.entries  = {}
    t.by_abbr  = {}
    t.experts  = {}
    t.root     = nil
    return t
end

--=======================================================================
-- Proto, Dissector, DissectorTable
--=======================================================================

-- Which dissectors got called, and with how many bytes: the payload
-- hand-off is a thing the test checks, since handing on the ethernet
-- padding instead of the frame is exactly the confusion the sniffer's
-- header exists to prevent.
M.calls = {}

local function dissector_for(name)
    return {
        name = name,
        call = function (_, tvb, _pinfo, _tree)
            table.insert(M.calls, { name = name, len = tvb:len(),
                                    bytes = tvb:bytes() })
            return tvb:len()
        end,
    }
end

M.Dissector = {
    get  = function (name) return dissector_for(name) end,
    list = function () return {} end,
}

-- What a dissector registered itself on, so the test can check it bound
-- to the ethertype the preference names.
M.registrations = {}

M.DissectorTable = {
    get = function (tablename)
        return {
            add = function (_, pattern, proto)
                table.insert(M.registrations,
                             { table = tablename, pattern = pattern,
                               proto = proto })
            end,
            remove = function () end,
        }
    end,
}

M.protos = {}

function M.Proto(name, description)
    local p = {
        is_proto    = true,
        name_abbr   = name,
        description = description,
        fields      = {},
        experts     = {},
        prefs       = {},
    }
    M.protos[name] = p
    return p
end

--=======================================================================
-- Installing the stub as globals
--=======================================================================

function M.install()
    _G.Proto          = M.Proto
    _G.ProtoField     = M.ProtoField
    _G.ProtoExpert    = M.ProtoExpert
    _G.Pref           = M.Pref
    _G.NSTime         = M.NSTime
    _G.Dissector      = M.Dissector
    _G.DissectorTable = M.DissectorTable
    _G.base           = M.base
    _G.expert         = M.expert
    _G.Tvb            = M.Tvb
end

function M.reset()
    M.calls         = {}
    M.registrations = {}
end

return M
