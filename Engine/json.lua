-- Strict startup-only JSON decoder. No eval, duplicate keys, NaN or Infinity.
local M = { null = {} }
local function utf8(n)
    if n < 128 then return string.char(n) end
    if n < 2048 then return string.char(192 + math.floor(n / 64), 128 + n % 64) end
    if n < 65536 then return string.char(224 + math.floor(n / 4096), 128 + math.floor(n / 64) % 64, 128 + n % 64) end
    return string.char(240 + math.floor(n / 262144), 128 + math.floor(n / 4096) % 64, 128 + math.floor(n / 64) % 64, 128 + n % 64)
end
function M.decode(text)
    assert(type(text) == "string" and #text <= 2097152, "JSON must be a string <= 2 MiB")
    local pos, depth = 1, 0
    local function fail(message) error("JSON byte " .. pos .. ": " .. message, 0) end
    local function skip() pos = text:find("[^ \t\r\n]", pos) or (#text + 1) end
    local function hex()
        local value = text:sub(pos, pos + 3)
        if #value ~= 4 or not value:match("^%x%x%x%x$") then fail("invalid unicode escape") end
        pos = pos + 4
        return tonumber(value, 16)
    end
    local escapes = { ['"']='"', ['\\']='\\', ['/']='/', b='\b', f='\f', n='\n', r='\r', t='\t' }
    local function str()
        pos = pos + 1
        local out = {}
        while pos <= #text do
            local c = text:sub(pos, pos)
            pos = pos + 1
            if c == '"' then return table.concat(out) end
            if c == '\\' then
                c = text:sub(pos, pos); pos = pos + 1
                if c == 'u' then
                    local n = hex()
                    if n >= 55296 and n <= 56319 then
                        if text:sub(pos, pos + 1) ~= '\\u' then fail("missing low surrogate") end
                        pos = pos + 2
                        local low = hex()
                        if low < 56320 or low > 57343 then fail("invalid low surrogate") end
                        n = 65536 + (n - 55296) * 1024 + low - 56320
                    elseif n >= 56320 and n <= 57343 then fail("unpaired surrogate") end
                    out[#out + 1] = utf8(n)
                else
                    if not escapes[c] then fail("invalid escape") end
                    out[#out + 1] = escapes[c]
                end
            else
                if c:byte() < 32 then fail("unescaped control character") end
                out[#out + 1] = c
            end
        end
        fail("unterminated string")
    end
    local parse
    parse = function()
        skip(); depth = depth + 1
        if depth > 128 then fail("nesting limit") end
        local c, value = text:sub(pos, pos), nil
        if c == '"' then value = str()
        elseif c == '{' or c == '[' then
            local object, close = c == '{', c == '{' and '}' or ']'
            value = {}; pos = pos + 1; skip()
            if text:sub(pos, pos) ~= close then
                while true do
                    local key = #value + 1
                    if object then
                        if text:sub(pos, pos) ~= '"' then fail("expected object key") end
                        key = str(); skip()
                        if value[key] ~= nil then fail("duplicate key " .. key) end
                        if text:sub(pos, pos) ~= ':' then fail("expected colon") end
                        pos = pos + 1
                    end
                    value[key] = parse(); skip()
                    local separator = text:sub(pos, pos)
                    if separator == close then break end
                    if separator ~= ',' then fail("expected comma or closing delimiter") end
                    pos = pos + 1; skip()
                end
            end
            pos = pos + 1
        elseif text:sub(pos, pos + 3) == 'true' then value = true; pos = pos + 4
        elseif text:sub(pos, pos + 4) == 'false' then value = false; pos = pos + 5
        elseif text:sub(pos, pos + 3) == 'null' then value = M.null; pos = pos + 4
        else
            local start = pos
            if c == '-' then pos = pos + 1 end
            c = text:sub(pos, pos)
            if c == '0' then pos = pos + 1
            elseif c:match('[1-9]') then
                repeat pos = pos + 1 until not text:sub(pos, pos):match('%d')
            else fail("expected value") end
            if text:sub(pos, pos) == '.' then
                pos = pos + 1
                if not text:sub(pos, pos):match('%d') then fail("expected fractional digit") end
                repeat pos = pos + 1 until not text:sub(pos, pos):match('%d')
            end
            c = text:sub(pos, pos)
            if c == 'e' or c == 'E' then
                pos = pos + 1; c = text:sub(pos, pos)
                if c == '+' or c == '-' then pos = pos + 1 end
                if not text:sub(pos, pos):match('%d') then fail("expected exponent digit") end
                repeat pos = pos + 1 until not text:sub(pos, pos):match('%d')
            end
            value = tonumber(text:sub(start, pos - 1))
            if not value or value == math.huge or value == -math.huge then fail("non-finite number") end
        end
        depth = depth - 1
        return value
    end
    local result = parse(); skip()
    if pos <= #text then fail("trailing content") end
    return result
end
return M
