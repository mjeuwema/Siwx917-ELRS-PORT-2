-- TNS|ELRS+mLRS|TNE
---- #########################################################################
---- # Combined ExpressLRS + native mLRS configurator                        #
---- # TX settings stay on the ELRS page (rate, band, power, Air Protocol).  #
---- # [mLRS Setup] is Edit Rx + Save over the mLRS air link.                #
---- # Copy to /SCRIPTS/TOOLS/ELRS-mLRS.lua                                  #
---- #########################################################################
---- # ExpressLRS portion: OpenTX / ExpressLRS, GPLv2                        #
---- # mLRS MBridge portion: MLRS project, GPL3                              #
---- #########################################################################
local EXITVER = "-- EXIT (Lua r18) --"
local MB = {}
local BP = {}
local tlmRedrawTimeout = 0
local titleH = 0
local tlmH = 0
local deviceId = 0xEE
local handsetId = 0xEF
local deviceName = nil
local currentFolderName = nil
local lineIndex = 1
local pageOffset = 0
local edit = nil
local fieldPopup
local fieldTimeout = 0
local loadQ = {}
local fieldChunk = 0
local fieldData = nil
local fields = {}
local devices = {}
local goodBadPkt = ""
local elrsFlags = 0
local elrsFlagsInfo = ""
local fields_count = 0
local devicesRefreshTimeout = 50
local currentFolderId = nil
local commandRunningIndicator = 1
local expectChunksRemain = -1
local deviceIsELRS_TX = nil
local linkstatTimeout = 100
local titleShowWarn = nil
local titleShowWarnTimeout = 100
local exitscript = 0

local COL1
local COL2
local maxLineIndex
local textYoffset
local textSize
local barTextSpacing

local function allocateFields()
  -- fields table is real fields, then Other Devices, mLRS Setup, devices, Exit/Back
  fields = {}
  for i=1, fields_count do
    fields[i] = { }
  end
  fields[#fields+1] = {id=fields_count+1, name="Other Devices", parent=255, type=16}
  fields[#fields+1] = {name="Bind Phrase", type=19}
  fields[#fields+1] = {name="mLRS Setup", type=18}
  fields[#fields+1] = {name=EXITVER, type=14}
end

local function createDeviceFields() -- put other devices in the field list
  local exitFld = {name=EXITVER, type=14}
  local mlrsFld = {name="mLRS Setup", type=18}
  local bindFld = {name="Bind Phrase", type=19}
  for i = 1, #fields do
    if fields[i].type == 14 then exitFld = fields[i] end
    if fields[i].type == 18 then mlrsFld = fields[i] end
    if fields[i].type == 19 then bindFld = fields[i] end
  end
  for i=1, #devices do
    local parent = (devices[i].id == deviceId) and 255 or (fields_count+1)
    fields[fields_count + 1 + i] = {id=devices[i].id, name=devices[i].name, parent=parent, type=15}
  end
  fields[fields_count + #devices + 2] = bindFld
  fields[fields_count + #devices + 3] = mlrsFld
  fields[fields_count + #devices + 4] = exitFld
end

local function reloadAllField()
  fieldTimeout = 0
  fieldChunk = 0
  fieldData = nil
  -- loadQ is actually a stack
  loadQ = {}
  for fieldId = fields_count, 1, -1 do
    loadQ[#loadQ+1] = fieldId
  end
end

local function getField(line)
  local counter = 1
  for i = 1, #fields do
    local field = fields[i]
    if currentFolderId == field.parent and not field.hidden then
      if counter < line then
        counter = counter + 1
      else
        return field
      end
    end
  end
end

local function incrField(step)
  local field = getField(lineIndex)
  local min, max = 0, 0
  if field.type <= 8 then
    min = field.min or 0
    max = field.max or 0
    step = (field.step or 1) * step
  elseif field.type == 9 then
    min = 0
    max = #field.values - 1
  end

  local newval = field.value
  repeat
    newval = newval + step
    if newval < min then
      newval = min
    elseif newval > max then
      newval = max
    end

    -- keep looping until a non-blank selection value is found
    if field.values == nil or #field.values[newval+1] ~= 0 then
      field.value = newval
      return
    end
  until (newval == min or newval == max)
end

-- Select the next or previous editable field
local function selectField(step)
  local newLineIndex = lineIndex
  local field
  repeat
    newLineIndex = newLineIndex + step
    if newLineIndex <= 0 then
      newLineIndex = #fields
    elseif newLineIndex == 1 + #fields then
      newLineIndex = 1
      pageOffset = 0
    end
    field = getField(newLineIndex)
  until newLineIndex == lineIndex or (field and field.name)
  lineIndex = newLineIndex
  if lineIndex > maxLineIndex + pageOffset then
    pageOffset = lineIndex - maxLineIndex
  elseif lineIndex <= pageOffset then
    pageOffset = lineIndex - 1
  end
end

local function fieldGetStrOrOpts(data, offset, last, isOpts)
  -- For isOpts: Split a table of byte values (string) with ; separator into a table
  -- Else just read a string until the first null byte
  local r = last or (isOpts and {})
  local opt = ''
  local vcnt = 0
  repeat
    local b = data[offset]
    offset = offset + 1

    if not last then
      if r and (b == 59 or b == 0) then -- ';'
        r[#r+1] = opt
        if opt ~= '' then
          vcnt = vcnt + 1
          opt = ''
        end
      elseif b ~= 0 then
        -- On firmwares that have constants defined for the arrow chars, use them in place of
        -- the \xc0 \xc1 chars (which are OpenTX-en)
        -- Use the table to convert the char, else use string.char if not in the table
        opt = opt .. (({
          [192] = CHAR_UP or (__opentx and __opentx.CHAR_UP),
          [193] = CHAR_DOWN or (__opentx and __opentx.CHAR_DOWN)
        })[b] or string.char(b))
      end
    end
  until b == 0

  return (r or opt), offset, vcnt, collectgarbage("collect")
end

local function getDevice(id)
  for _, device in ipairs(devices) do
    if device.id == id then
      return device
    end
  end
end

local function fieldGetValue(data, offset, size)
  local result = 0
  for i=0, size-1 do
    result = bit32.lshift(result, 8) + data[offset + i]
  end
  return result
end

local function reloadCurField()
  local field = getField(lineIndex)
  fieldTimeout = 0
  fieldChunk = 0
  fieldData = nil
  loadQ[#loadQ+1] = field.id
end

-- UINT8/INT8/UINT16/INT16 + FLOAT + TEXTSELECT
local function fieldUnsignedLoad(field, data, offset, size, unitoffset)
  field.value = fieldGetValue(data, offset, size)
  field.min = fieldGetValue(data, offset+size, size)
  field.max = fieldGetValue(data, offset+2*size, size)
  --field.default = fieldGetValue(data, offset+3*size, size)
  field.unit = fieldGetStrOrOpts(data, offset+(unitoffset or (4*size)), field.unit)
  -- Only store the size if it isn't 1 (covers most fields / selection)
  if size ~= 1 then
    field.size = size
  end
end

local function fieldUnsignedToSigned(field, size)
  local bandval = bit32.lshift(0x80, (size-1)*8)
  field.value = field.value - bit32.band(field.value, bandval) * 2
  field.min = field.min - bit32.band(field.min, bandval) * 2
  field.max = field.max - bit32.band(field.max, bandval) * 2
  --field.default = field.default - bit32.band(field.default, bandval) * 2
end

local function fieldSignedLoad(field, data, offset, size, unitoffset)
  fieldUnsignedLoad(field, data, offset, size, unitoffset)
  fieldUnsignedToSigned(field, size)
  -- signed ints are INTdicated by a negative size
  field.size = -size
end

local function fieldIntLoad(field, data, offset)
  -- Type is U8/I8/U16/I16, use that to determine the size and signedness
  local loadFn = (field.type % 2 == 0) and fieldUnsignedLoad or fieldSignedLoad
  loadFn(field, data, offset, math.floor(field.type / 2) + 1)
end

local function fieldIntSave(field)
  local value = field.value
  local size = field.size or 1
  -- Convert signed to 2s complement
  if size < 0 then
    size = -size
    if value < 0 then
      value = bit32.lshift(0x100, (size-1)*8) + value
    end
  end

  local frame = { deviceId, handsetId, field.id }
  for i = size-1, 0, -1 do
    frame[#frame + 1] = bit32.rshift(value, 8*i) % 256
  end
  crossfireTelemetryPush(0x2D, frame)
end

local function fieldIntDisplay(field, y, attr)
  lcd.drawText(COL2, y, field.value .. field.unit, attr)
end

-- -- FLOAT
local function fieldFloatLoad(field, data, offset)
  fieldSignedLoad(field, data, offset, 4, 21)
  field.prec = data[offset+16]
  if field.prec > 3 then
    field.prec = 3
  end
  field.step = fieldGetValue(data, offset+17, 4)

  -- precompute the format string to preserve the precision
  field.fmt = "%." .. tostring(field.prec) .. "f" .. field.unit
  -- Convert precision to a divider
  field.prec = 10 ^ field.prec
end

local function fieldFloatDisplay(field, y, attr)
  lcd.drawText(COL2, y, string.format(field.fmt, field.value / field.prec), attr)
end

-- TEXT SELECTION
local function fieldTextSelLoad(field, data, offset)
  local vcnt
  local cached = field.nc == nil and field.values
  field.values, offset, vcnt = fieldGetStrOrOpts(data, offset, cached, true)
  -- 'Disable' the line if values only has one option in the list
  if not cached then
    field.grey = vcnt <= 1
  end
  field.value = data[offset]
  -- min max and default (offset+1 to 3) are not used on selections
  -- units never uses cache
  field.unit = fieldGetStrOrOpts(data, offset+4)
  field.nc = nil -- use cache next time
end

local function fieldTextSelDisplay_color(field, y, attr, color)
  local val = field.values[field.value+1] or "ERR"
  lcd.drawText(COL2, y, val, attr + color)
  local strPix = lcd.sizeText and lcd.sizeText(val) or (10 * #val)
  lcd.drawText(COL2 + strPix, y, field.unit, color)
end

local function fieldTextSelDisplay_bw(field, y, attr)
  lcd.drawText(COL2, y, field.values[field.value+1] or "ERR", attr)
  lcd.drawText(lcd.getLastPos(), y, field.unit, 0)
end

-- STRING
local function fieldStringLoad(field, data, offset)
  field.value, offset = fieldGetStrOrOpts(data, offset)
  if #data >= offset then
    field.maxlen = data[offset]
  end
end

local function fieldStringDisplay(field, y, attr)
  lcd.drawText(COL2, y, field.value, attr)
end

local function fieldFolderOpen(field)
  currentFolderId = field.id
  currentFolderName = field.name

  local backFld = fields[#fields]
  backFld.name = "----BACK----"
  -- Store the lineIndex and pageOffset to return to in the backFld
  backFld.li = lineIndex
  backFld.po = pageOffset
  backFld.parent = currentFolderId
  backFld.grandParent = fields[currentFolderId].parent

  lineIndex = 1
  pageOffset = 0
end

local function fieldFolderDeviceOpen(field)
  -- crossfireTelemetryPush(0x28, { 0x00, 0xEA }) --broadcast with standard handset ID to get all node respond correctly
  -- Make sure device fields are in the folder when it opens
  createDeviceFields()
  return fieldFolderOpen(field)
end

local function fieldFolderDisplay(field,y ,attr)
  lcd.drawText(COL1, y, "> " .. field.name, attr + BOLD)
end

local function fieldCommandLoad(field, data, offset)
  field.status = data[offset]
  field.timeout = data[offset+1]
  field.info = fieldGetStrOrOpts(data, offset+2)
  if field.status == 0 then
    fieldPopup = nil
  end
end

local function fieldCommandSave(field)
  reloadCurField()

  if field.status ~= nil then
    if field.status < 4 then
      field.status = 1
      crossfireTelemetryPush(0x2D, { deviceId, handsetId, field.id, field.status })
      fieldPopup = field
      fieldPopup.lastStatus = 0
      fieldTimeout = getTime() + field.timeout
    end
  end
end

local function fieldCommandDisplay(field, y, attr)
    lcd.drawText(10, y, "[" .. field.name .. "]", attr + BOLD)
end

local function fieldBackExec(field)
  if field.grandParent then -- Back from Sub-menu
    fieldFolderOpen(fields[field.grandParent])
  elseif field.parent then
    lineIndex = field.li or 1
    pageOffset = field.po or 0

    field.name = EXITVER
    field.parent = nil
    field.li = nil
    field.po = nil
    currentFolderId = nil
    currentFolderName = nil
  else -- Executing EXIT
    exitscript = 1
  end
end

local function changeDeviceId(devId) --change to selected device ID
  local device = getDevice(devId)
  if deviceId == devId and fields_count == device.fldcnt then return end

  deviceId = devId
  elrsFlags = 0
  currentFolderId = nil
  currentFolderName = nil
  deviceName = device.name
  fields_count = device.fldcnt
  deviceIsELRS_TX = device.isElrs and devId == 0xEE or nil -- ELRS and ID is TX module
  handsetId = deviceIsELRS_TX and 0xEF or 0xEA -- Address ELRS_LUA vs RADIO_TRANSMITTER

  allocateFields()
  reloadAllField()
end

local function fieldDeviceIdSelect(field)
  return changeDeviceId(field.id)
end

local function parseDeviceInfoMessage(data)
  local id = data[2]
  local newName, offset = fieldGetStrOrOpts(data, 3)
  local device = getDevice(id)
  if device == nil then
    device = { id = id }
    devices[#devices + 1] = device
  end
  device.name = newName
  device.fldcnt = data[offset + 12]
  device.isElrs = fieldGetValue(data, offset, 4) == 0x454C5253 -- SerialNumber = 'E L R S'

  if deviceId == id then
    changeDeviceId(id)
  end
  -- DeviceList change while in Other Devices, refresh list
  if currentFolderId == fields_count + 1 then
    createDeviceFields()
  end
end

local functions = {
  { load=fieldIntLoad, save=fieldIntSave, display=fieldIntDisplay }, --1 UINT8(0)
  { load=fieldIntLoad, save=fieldIntSave, display=fieldIntDisplay }, --2 INT8(1)
  { load=fieldIntLoad, save=fieldIntSave, display=fieldIntDisplay }, --3 UINT16(2)
  { load=fieldIntLoad, save=fieldIntSave, display=fieldIntDisplay }, --4 INT16(3)
  nil,
  nil,
  nil,
  nil,
  { load=fieldFloatLoad, save=fieldIntSave, display=fieldFloatDisplay },  --9 FLOAT(8)
  { load=fieldTextSelLoad, save=fieldIntSave, display=nil }, --10 SELECT(9)
  { load=fieldStringLoad, save=nil, display=fieldStringDisplay }, --11 STRING(10) editing NOTIMPL
  { load=nil, save=fieldFolderOpen, display=fieldFolderDisplay }, --12 FOLDER(11)
  { load=fieldStringLoad, save=nil, display=fieldStringDisplay }, --13 INFO(12)
  { load=fieldCommandLoad, save=fieldCommandSave, display=fieldCommandDisplay }, --14 COMMAND(13)
  { load=nil, save=fieldBackExec, display=fieldCommandDisplay }, --15 back/exit(14)
  { load=nil, save=fieldDeviceIdSelect, display=fieldCommandDisplay }, --16 device(15)
  { load=nil, save=fieldFolderDeviceOpen, display=fieldFolderDisplay }, --17 deviceFOLDER(16)
  nil, --18 unused type 17
  { load=nil, save=nil, display=fieldCommandDisplay }, --19 [mLRS Setup](18)
  { load=nil, save=nil, display=fieldCommandDisplay }, --20 [Bind Phrase](19)
}

local function parseParameterInfoMessage(data)
  local fieldId = (fieldPopup and fieldPopup.id) or loadQ[#loadQ]
  if data[2] ~= deviceId or data[3] ~= fieldId then
    fieldData = nil
    fieldChunk = 0
    return
  end
  local field = fields[fieldId]
  local chunksRemain = data[4]
  -- If no field or the chunksremain changed when we have data, don't continue
  if not field or (fieldData and chunksRemain ~= expectChunksRemain) then
    return
  end

  local offset
  -- If data is chunked, copy it to persistent buffer
  if chunksRemain > 0 or fieldChunk > 0 then
    fieldData = fieldData or {}
    for i=5, #data do
      fieldData[#fieldData + 1] = data[i]
      data[i] = nil
    end
    offset = 1
  else
    -- All data arrived in one chunk, operate directly on data
    fieldData = data
    offset = 5
  end

  if chunksRemain > 0 then
    fieldChunk = fieldChunk + 1
    expectChunksRemain = chunksRemain - 1
  else
    -- Field data stream is now complete, process into a field
    loadQ[#loadQ] = nil

    if #fieldData > (offset + 2) then
      field.id = fieldId
      field.parent = (fieldData[offset] ~= 0) and fieldData[offset] or nil
      field.type = bit32.band(fieldData[offset+1], 0x7f)
      field.hidden = bit32.btest(fieldData[offset+1], 0x80) or nil
      field.name, offset = fieldGetStrOrOpts(fieldData, offset+2, field.name)
      local fn = functions[field.type+1]
      if fn and fn.load then
        fn.load(field, fieldData, offset)
      end
      if field.min == 0 then field.min = nil end
      if field.max == 0 then field.max = nil end
    end

    fieldChunk = 0
    fieldData = nil

    -- Return value is if the screen should be updated
    -- If deviceId is TX module, then the Bad/Good drives the update; for other
    -- devices update each new item. and always update when the queue empties
    return deviceId ~= 0xEE or #loadQ == 0
  end
end

local function parseElrsInfoMessage(data)
  if data[2] ~= deviceId then
    fieldData = nil
    fieldChunk = 0
    return
  end

  local badPkt = data[3]
  local goodPkt = (data[4]*256) + data[5]
  local newFlags = data[6]
  -- If flags are changing, reset the warning timeout to display/hide message immediately
  if newFlags ~= elrsFlags then
    elrsFlags = newFlags
    titleShowWarnTimeout = 0
  end
  elrsFlagsInfo = fieldGetStrOrOpts(data, 7)

  local state = (bit32.btest(elrsFlags, 1) and "C") or "-"
  goodBadPkt = string.format("%u/%u   %s", badPkt, goodPkt, state)
end

local function parseElrsV1Message(data)
  if (data[1] ~= 0xEA) or (data[2] ~= 0xEE) then
    return
  end

  -- local badPkt = data[9]
  -- local goodPkt = (data[10]*256) + data[11]
  -- goodBadPkt = string.format("%u/%u   X", badPkt, goodPkt)
  fieldPopup = {id = 0, status = 2, timeout = 0xFF, info = "ERROR: 1.x firmware"}
  fieldTimeout = getTime() + 0xFFFF
end

local function refreshNext(skipPush)
  local command, data, forceRedraw
  repeat
    command, data = crossfireTelemetryPop()
    if command == 0x29 then
      parseDeviceInfoMessage(data)
    elseif command == 0x2B then
      if parseParameterInfoMessage(data) then
        forceRedraw = true
      end
      if #loadQ > 0 then
        fieldTimeout = 0 -- request next chunk immediately
      elseif fieldPopup then
        fieldTimeout = getTime() + fieldPopup.timeout
      end
    elseif command == 0x2D then
      parseElrsV1Message(data)
    elseif command == 0x2E then
      parseElrsInfoMessage(data)
      forceRedraw = true
    end
  until command == nil

  -- Don't even bother with return value, skipPush implies redraw
  if skipPush then return end

  local time = getTime()
  if fieldPopup then
    if time > fieldTimeout and fieldPopup.status ~= 3 then
      crossfireTelemetryPush(0x2D, { deviceId, handsetId, fieldPopup.id, 6 }) -- lcsQuery
      fieldTimeout = time + fieldPopup.timeout
    end
  elseif time > devicesRefreshTimeout and #devices == 0 then
    forceRedraw = true -- handles initial screen draw
    devicesRefreshTimeout = time + 100 -- 1s
    crossfireTelemetryPush(0x28, { 0x00, 0xEA })
  elseif time > linkstatTimeout then
    if deviceIsELRS_TX then
      crossfireTelemetryPush(0x2D, { deviceId, handsetId, 0x0, 0x0 }) --request linkstat
    else
      goodBadPkt = ""
    end
    linkstatTimeout = time + 100
  elseif time > fieldTimeout and fields_count ~= 0 then
    if #loadQ > 0 then
      crossfireTelemetryPush(0x2C, { deviceId, handsetId, loadQ[#loadQ], fieldChunk })
      fieldTimeout = time + (deviceIsELRS_TX and 50 or 500) -- 0.5s for local / 5s for remote devices
    end
  end

  if time > titleShowWarnTimeout then
    -- if elrsFlags bit set is bit higher than bit 0 and bit 1, it is warning flags
    titleShowWarn = (elrsFlags > 3 and not titleShowWarn) or nil
    titleShowWarnTimeout = time + 100
    forceRedraw = true
  end
  if fields_count ~= 0 and time > tlmRedrawTimeout then
    tlmRedrawTimeout = time + 20
    forceRedraw = true
  end

  return forceRedraw
end

local function tlm_sensor(names)
  for i = 1, #names do
    local ok, fi
    if getFieldInfo then
      ok, fi = pcall(getFieldInfo, names[i])
      if ok and fi then
        local vok, v = pcall(getValue, fi.id)
        if vok then return v end
      end
    else
      ok, fi = pcall(getValue, names[i])
      if ok and fi ~= nil then
        return fi
      end
    end
  end
  return nil
end

local function tlm_proto()
  if MB.active then
    return "mLRS"
  end
  for i = 1, (fields_count or 0) do
    local f = fields[i]
    if f and f.name == "Air Protocol" and f.values ~= nil and f.value ~= nil then
      return f.values[f.value + 1] or "ELRS"
    end
  end
  return "ELRS"
end

local function fmt_Bps(bps)
  if bps == nil then
    return "--"
  end
  if bps >= 1000 then
    return string.format("%.1fkB/s", bps / 1000)
  end
  return string.format("%dB/s", math.floor(bps + 0.5))
end

-- Link payload capacity from air interval (band+rate). LQ scales delivered rate.
-- Do not use last mux fill: hopmask-only packets are ~9 B and look like 170 B/s.
local function tlm_payload_Bps(lq)
  if MB.info == nil then
    return nil, nil
  end
  local rate = MB.info.rate or 0
  local band = MB.info.band or 0
  if rate > 2 then rate = 1 end
  if band > 1 then band = 0 end
  local us915 = { [0] = 32000, [1] = 53000, [2] = 20000 }
  local us24  = { [0] = 20000, [1] = 32000, [2] = 53000 }
  local us = (band == 1) and us24[rate] or us915[rate]
  if us == nil or us == 0 then
    return nil, nil
  end
  local hz = 1000000 / us
  local tight = (band == 0 and rate == 2) or (band == 1 and rate == 0)
  local scale = 1
  if lq ~= nil and lq > 0 then
    scale = lq / 100
  end
  local ul = hz * 52 * scale
  local dl = (tight and (hz / 2) or hz) * 70 * scale
  return ul, dl
end

local function tlm_text()
  local rssi = tlm_sensor({"1RSS", "RSSI"})
  if rssi == nil then
    rssi = getRSSI()
  end
  local lq = tlm_sensor({"RQly"})
  local snr = tlm_sensor({"RSNR", "SNR"})
  local pwr = tlm_sensor({"TPWR"})
  local rfmd = tlm_sensor({"RFMD"})
  local proto = tlm_proto()
  local rssi_s = (rssi ~= nil) and string.format("%d", rssi) or "--"
  local lq_s = (lq ~= nil) and string.format("%d", lq) or "--"
  local snr_s = (snr ~= nil) and string.format("%d", snr) or "--"
  local pwr_s = "--"
  if MB.info ~= nil and MB.info.tx_dbm ~= nil then
    pwr_s = string.format("%ddBm", MB.info.tx_dbm)
  elseif pwr ~= nil then
    pwr_s = string.format("%dmW", pwr)
  end
  local ul_bps, dl_bps = tlm_payload_Bps(lq)
  if lcd.RGB == nil then
    local spd = (ul_bps ~= nil) and (" " .. fmt_Bps(ul_bps)) or ""
    return string.format("%s %s %s%% %sdB %s%s", proto, rssi_s, lq_s, snr_s, pwr_s, spd), lq
  end
  local extra = ""
  if ul_bps ~= nil then
    if LCD_W ~= nil and LCD_W >= 480 then
      extra = string.format("  UL %s  DL %s", fmt_Bps(ul_bps), fmt_Bps(dl_bps))
    else
      extra = string.format("  %s", fmt_Bps(ul_bps))
    end
  end
  if MB.info ~= nil and (MB.info.hop_count or 0) > 0 then
    extra = extra .. string.format("  skip %d/%d", MB.info.hop_skip or 0, MB.info.hop_count)
  end
  if rfmd ~= nil and LCD_W ~= nil and LCD_W >= 480 then
    extra = extra .. string.format("  RFMD %d", rfmd)
  end
  return string.format("%s  RSSI %s  LQ %s  SNR %s  %s%s", proto, rssi_s, lq_s, snr_s, pwr_s, extra), lq
end

local function drawTlmStrip(y, h)
  local txt, lq = tlm_text()
  h = h or tlmH
  if h == 0 then h = (lcd.RGB ~= nil) and 18 or 9 end
  if lcd.RGB ~= nil then
    local bg = lcd.RGB(0x3a, 0x3a, 0x48)
    if lq ~= nil then
      if lq >= 80 then
        bg = lcd.RGB(0x1f, 0x5c, 0x38)
      elseif lq >= 50 then
        bg = lcd.RGB(0x6a, 0x55, 0x12)
      else
        bg = lcd.RGB(0x6a, 0x22, 0x22)
      end
    end
    lcd.setColor(CUSTOM_COLOR, bg)
    lcd.drawFilledRectangle(0, y, LCD_W, h, CUSTOM_COLOR)
    lcd.setColor(CUSTOM_COLOR, lcd.RGB(0xff, 0xff, 0xff))
    lcd.drawText(COL1 or 4, y + math.max(0, math.floor((h - (textSize or 16)) / 2)), txt, CUSTOM_COLOR + SMLSIZE)
  else
    lcd.drawFilledRectangle(0, y, LCD_W, h, GREY_DEFAULT)
    lcd.drawText(0, y, txt, INVERS + SMLSIZE)
  end
end

local lcd_title -- holds function that is color/bw version
local function lcd_title_color()
  lcd.clear()

  local EBLUE = lcd.RGB(0x43, 0x61, 0xAA)
  local EGREEN = lcd.RGB(0x9f, 0xc7, 0x6f)
  local EGREY1 = lcd.RGB(0x91, 0xb2, 0xc9)
  local EGREY2 = lcd.RGB(0x6f, 0x62, 0x7f)

  -- Field display area (white w/ 2px green border)
  lcd.setColor(CUSTOM_COLOR, EGREEN)
  lcd.drawRectangle(0, 0, LCD_W, LCD_H, CUSTOM_COLOR)
  lcd.drawRectangle(1, 0, LCD_W - 2, LCD_H - 1, CUSTOM_COLOR)
  -- title bar (name) + live telemetry strip
  local th = titleH
  if th == 0 then th = textSize + barTextSpacing + barTextSpacing end
  lcd.drawFilledRectangle(0, 0, LCD_W, th, CUSTOM_COLOR)
  lcd.setColor(CUSTOM_COLOR, EGREY1)
  lcd.drawFilledRectangle(LCD_W - textSize, 0, textSize, th, CUSTOM_COLOR)
  lcd.setColor(CUSTOM_COLOR, EGREY2)
  lcd.drawRectangle(LCD_W - textSize, 0, textSize, th - 1, CUSTOM_COLOR)
  lcd.drawRectangle(LCD_W - textSize, 1 , textSize - 1, th - 2, CUSTOM_COLOR)
  lcd.setColor(CUSTOM_COLOR, BLACK)
  if titleShowWarn then
    lcd.drawText(COL1 + 1, barTextSpacing, elrsFlagsInfo, CUSTOM_COLOR)
  else
    lcd.drawText(COL1 + 1, barTextSpacing, deviceName, CUSTOM_COLOR)
    lcd.drawText(LCD_W / 2, barTextSpacing, currentFolderName or "", CENTER + BOLD + CUSTOM_COLOR)
    lcd.drawText(LCD_W - 5, barTextSpacing, goodBadPkt, RIGHT + BOLD + CUSTOM_COLOR)
  end
  if #loadQ > 0 and fields_count > 0 then
    local barW = (COL2-4) * (fields_count - #loadQ) / fields_count
    lcd.setColor(CUSTOM_COLOR, EBLUE)
    lcd.drawFilledRectangle(2, th - 4, barW, 3, CUSTOM_COLOR)
  end
  drawTlmStrip(th, tlmH)
end

local function lcd_title_bw()
  lcd.clear()
  local bh = 9
  if not titleShowWarn then
    lcd.drawText(LCD_W - 1, 1, goodBadPkt, RIGHT)
    lcd.drawLine(LCD_W - 10, 0, LCD_W - 10, bh-1, SOLID, INVERS)
  end

  if #loadQ > 0 and fields_count > 0 then
    lcd.drawFilledRectangle(COL2, 0, LCD_W, bh, GREY_DEFAULT)
    lcd.drawGauge(0, 0, COL2, bh, fields_count - #loadQ, fields_count, 0)
  else
    lcd.drawFilledRectangle(0, 0, LCD_W, bh, GREY_DEFAULT)
    if titleShowWarn then
      lcd.drawText(COL1, 1, elrsFlagsInfo, INVERS)
    else
      lcd.drawText(COL1, 1, currentFolderName or deviceName, INVERS)
    end
  end
  drawTlmStrip(bh, 9)
end

local function lcd_warn()
  lcd.drawText(COL1, textSize*2, "Error:")
  lcd.drawText(COL1, textSize*3, elrsFlagsInfo)
  lcd.drawText(LCD_W/2, textSize*5, "[OK]", BLINK + INVERS + CENTER)
end

local function reloadRelatedFields(field)
  -- Reload the parent folder to update the description
  if field.parent then
    loadQ[#loadQ+1] = field.parent
    fields[field.parent].name = nil
  end

  -- Reload all editable fields at the same level as well as the parent item
  for fieldId = fields_count, 1, -1 do
    -- Skip this field, will be added to end
    local fldTest = fields[fieldId]
    local fldType = fldTest.type or 99 -- type could be nil if still loading
    if fieldId ~= field.id
      and fldTest.parent == field.parent
      and (fldType < 11 or fldType == 12 or fldType == 13) then -- ignores FOLDER/devices/EXIT
      fldTest.nc = true -- "no cache" the options
      loadQ[#loadQ+1] = fieldId
    end
  end

  -- TEXTSELECT (BLE RemoteID, etc.): keep the value just saved. An OTA
  -- PARAMETER_READ can overtake the WRITE and snap the display back to Off.
  if field.type ~= 9 then
    loadQ[#loadQ+1] = field.id
  end
  -- RX writes travel over air; wait longer before any read-back.
  fieldTimeout = getTime() + ((deviceId ~= 0xEE) and 80 or 20)
  -- Also push the next bad/good update further out
  linkstatTimeout = fieldTimeout + 100
end

local function handleDevicePageEvent(event)
  if #fields == 0 then --if there is no field yet
    return
  else
    if fields[#fields].name == nil then --if back button is not assigned yet, means there is no field yet.
      return
    end
  end

  if event == EVT_VIRTUAL_EXIT then -- Cancel edit / go up a folder / reload all
    if edit then
      edit = nil
      reloadCurField()
    else
      if currentFolderId == nil and #loadQ == 0 then -- only do reload if we're in the root folder and finished loading
        if deviceId ~= 0xEE then
          changeDeviceId(0xEE)
        else
          reloadAllField()
        end
        crossfireTelemetryPush(0x28, { 0x00, 0xEA })
      else
        fieldBackExec(fields[#fields])
      end
    end
  elseif event == EVT_VIRTUAL_ENTER then -- toggle editing/selecting current field
    if elrsFlags > 0x1F then
      elrsFlags = 0
      crossfireTelemetryPush(0x2D, { deviceId, handsetId, 0x2E, 0x00 })
    else
      local field = getField(lineIndex)
      if field and field.name then
        -- Editable fields
        if not field.grey and field.type < 10 then
          edit = not edit
        end
        if field.type == 19 then
          BP.open()
          return
        end
        if field.type == 18 then
          MB.open()
          return
        end
        if not edit then
          if functions[field.type+1].save then
            functions[field.type+1].save(field)
          end
          if field.type and field.type < 10 then
            reloadRelatedFields(field)
          end
        end
      end
    end
  elseif edit then
    if event == EVT_VIRTUAL_NEXT then
      incrField(1)
    elseif event == EVT_VIRTUAL_PREV then
      incrField(-1)
    end
  else
    if event == EVT_VIRTUAL_NEXT then
      selectField(1)
    elseif event == EVT_VIRTUAL_PREV then
      selectField(-1)
    end
  end
end

-- Main
local function runDevicePage(event)
  handleDevicePageEvent(event)

  lcd_title()

  if #devices > 1 then -- show other device folder
    fields[fields_count+1].parent = nil
  end
  if elrsFlags > 0x1F then
    lcd_warn()
  else
    for y = 1, maxLineIndex+1 do
      local field = getField(pageOffset+y)
      if not field then
        break
      elseif field.name ~= nil then
        local attr = lineIndex == (pageOffset+y)
          and ((edit and BLINK or 0) + INVERS)
          or 0
        local color = field.grey and COLOR_THEME_DISABLED or 0
        if field.type < 11 or field.type == 12 then -- if not folder, command, or back
          lcd.drawText(COL1, y*textSize+textYoffset, field.name, color)
        end
        local fn = field.type and functions[field.type+1]
        if fn and fn.display then
          fn.display(field, y*textSize+textYoffset, attr, color)
        end
      end
    end
  end
end

local function popupCompat(t, m, e)
  -- Only use 2 of 3 arguments for older platforms
  return popupConfirmation(t, e)
end

local function runPopupPage(event)
  if event == EVT_VIRTUAL_EXIT then
    crossfireTelemetryPush(0x2D, { deviceId, handsetId, fieldPopup.id, 5 }) -- lcsCancel
    fieldTimeout = getTime() + 200 -- 2s
  end

  if fieldPopup.status == 0 and fieldPopup.lastStatus ~= 0 then -- stopped
      popupCompat(fieldPopup.info, "Stopped!", event)
      reloadAllField()
      fieldPopup = nil
  elseif fieldPopup.status == 3 then -- confirmation required
    local result = popupCompat(fieldPopup.info, "PRESS [OK] to confirm", event)
    fieldPopup.lastStatus = fieldPopup.status
    if result == "OK" then
      crossfireTelemetryPush(0x2D, { deviceId, handsetId, fieldPopup.id, 4 }) -- lcsConfirmed
      fieldTimeout = getTime() + fieldPopup.timeout -- we are expecting an immediate response
      fieldPopup.status = 4
    elseif result == "CANCEL" then
      fieldPopup = nil
    end
  elseif fieldPopup.status == 2 then -- running
    if fieldChunk == 0 then
      commandRunningIndicator = (commandRunningIndicator % 4) + 1
    end
    local result = popupCompat(fieldPopup.info .. " [" .. string.sub("|/-\\", commandRunningIndicator, commandRunningIndicator) .. "]", "Press [RTN] to exit", event)
    fieldPopup.lastStatus = fieldPopup.status
    if result == "CANCEL" then
      crossfireTelemetryPush(0x2D, { deviceId, handsetId, fieldPopup.id, 5 }) -- lcsCancel
      fieldTimeout = getTime() + fieldPopup.timeout -- we are expecting an immediate response
      fieldPopup = nil
    end
  end
end

local function touch2evt(event, touchState)
  -- Convert swipe events to normal events Left/Right/Up/Down -> EXIT/ENTER/PREV/NEXT
  -- PREV/NEXT are swapped if editing
  -- TAP is converted to ENTER
  touchState = touchState or {}
  return (touchState.swipeLeft and EVT_VIRTUAL_EXIT)
    or (touchState.swipeRight  and EVT_VIRTUAL_ENTER)
    or (touchState.swipeUp     and (edit and EVT_VIRTUAL_NEXT or EVT_VIRTUAL_PREV))
    or (touchState.swipeDown   and (edit and EVT_VIRTUAL_PREV or EVT_VIRTUAL_NEXT))
    or (event == EVT_TOUCH_TAP and EVT_VIRTUAL_ENTER)
end

local function setLCDvar()
  -- Set the title function depending on if LCD is color, and free the other function and
  -- set textselection unit function, use GetLastPost or sizeText
  if (lcd.RGB ~= nil) then
    lcd_title = lcd_title_color
    functions[10].display = fieldTextSelDisplay_color
  else
    lcd_title = lcd_title_bw
    functions[10].display = fieldTextSelDisplay_bw
    touch2evt = nil
  end
  lcd_title_color = nil
  lcd_title_bw = nil
  fieldTextSelDisplay_bw = nil
  fieldTextSelDisplay_color = nil
  -- Determine if popupConfirmation takes 3 arguments or 2
  -- if pcall(popupConfirmation, "", "", EVT_VIRTUAL_EXIT) then
  -- major 1 is assumed to be FreedomTX
  local _, _, major = getVersion()
  if major ~= 1 then
    popupCompat = popupConfirmation
  end

  if (lcd.RGB ~= nil) then
    local ver, radio, maj, minor, rev, osname = getVersion()

    if osname ~= nil and osname == "EdgeTX" then
      textWidth, textSize = lcd.sizeText("Qg") -- determine standard font height for EdgeTX
    else
      textSize = 21                            -- use this for OpenTX
    end

    COL1 = 3
    COL2 = LCD_W/2
    barTextSpacing = 4
    titleH = textSize + barTextSpacing + barTextSpacing
    tlmH = math.max(16, textSize - 2)
    barHeight = titleH + tlmH
    local gap = 2 * barTextSpacing + 2
    textYoffset = gap + tlmH
    maxLineIndex = math.floor(((LCD_H - barHeight - gap) / textSize)) - 1
  else
    if LCD_W == 212 then
      COL2 = 110
    else
      COL2 = 70
    end
    titleH = 9
    tlmH = 9
    barHeight = titleH + tlmH
    if LCD_H == 96 then
      maxLineIndex = 8
    else
      maxLineIndex = 5
    end
    COL1 = 0
    textYoffset = 3 + tlmH
    textSize = 8
  end
end

local function setMock()
  -- Setup fields to display if running in Simulator
  local _, rv = getVersion()
  if string.sub(rv, -5) ~= "-simu" then return end
  local mock = loadScript("mockup/elrsmock.lua")
  if mock == nil then return end
  fields, goodBadPkt, deviceName = mock()
  fields_count = #fields - 1
  loadQ = { fields_count }
  deviceIsELRS_TX = true
end

local function checkCrsfModule()
  -- Loop through the modules and look for one set to CRSF (5)
  for modIdx = 0, 1 do
    local mod = model.getModule(modIdx)
    if mod and (mod.Type == nil or mod.Type == 5) then
      -- CRSF found, put module type in Loading message
      local modDescrip = (mod.Type == nil) and " awaiting" or (modIdx == 0) and " Internal" or " External"
      -- Prefix with "Lua rXXX" from between EXITVER parens
      deviceName = string.match(EXITVER, "%((.*)%)") .. modDescrip .. " TX..."
      checkCrsfModule = nil
      return 0
    end
  end

  -- No CRSF module found, save an error message for run()
  lcd.clear()
  local y = 0
  lcd.drawText(2, y, "  No ExpressLRS", MIDSIZE)
  y = y + (textSize * 2) - 2
  local msgs = {
    " Enable a CRSF Internal",
    "   or External module in",
    "       Model settings",
    "  If module is internal",
    " also set Internal RF to",
    " CRSF in SYS->Hardware",
  }
  for i, msg in ipairs(msgs) do
    lcd.drawText(2, y, msg)
    y = y + textSize
    if i == 3 then
      lcd.drawLine(0, y, LCD_W, y, SOLID, INVERS)
      y = y + 2
    end
  end

  return 0
end

----------------------------------------------------------------------
-- ELRS bind phrase (stock 4.1 MSP 0x2D / BIND_PHRASE). ELRS page item.
----------------------------------------------------------------------
BP.active = false
BP.value = ""
BP.sidx = 0
BP.edit = false
BP.status = ""
local BP_CHARS = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_#-. "
local BP_MAX = 20

function BP.open()
  BP.active = true
  BP.edit = false
  BP.sidx = 0
  BP.status = "ENTER=apply to TX+RX"
  if BP.value == "" then BP.value = "expresslrs" end
end

local function bpSend(dest, phrase)
  local payload = { dest, 0xEA, 0x30, 1 + #phrase, 0x2D, 0x01 }
  for i = 1, #phrase do payload[#payload + 1] = string.byte(phrase, i) end
  crossfireTelemetryPush(0x7C, payload)
end

function BP.run(event)
  lcd.clear()
  lcd.drawFilledRectangle(0, 0, LCD_W, titleH, GREY_DEFAULT)
  lcd.drawText(COL1, barTextSpacing, "ELRS Bind Phrase", INVERS)
  lcd.drawText(COL1, titleH + 4, "Same as ELRS 4.1 Bind Manager", 0)
  local y = titleH + 4 + textSize
  local shown = BP.value
  if shown == "" then shown = "(empty)" end
  lcd.drawText(COL1, y, shown, BP.edit and INVERS or 0)
  if BP.edit then
    local pre = string.sub(BP.value, 1, BP.sidx)
    local cur = string.sub(BP.value .. " ", BP.sidx + 1, BP.sidx + 1)
    lcd.drawText(COL1, y + textSize, string.rep(" ", #pre) .. "^" .. cur, 0)
  end
  lcd.drawText(COL1, LCD_H - textSize * 2, BP.status or "", 0)
  lcd.drawText(COL1, LCD_H - textSize, "RTN=back  +/- char  ENTER=set", 0)

  if event == EVT_VIRTUAL_EXIT then
    if BP.edit then BP.edit = false else BP.active = false end
    return 0
  end
  if event == EVT_VIRTUAL_ENTER then
    if not BP.edit then
      BP.edit = true
      if #BP.value < 1 then BP.value = "a" end
      BP.sidx = math.min(BP.sidx, math.max(0, #BP.value - 1))
    else
      BP.edit = false
      local phrase = BP.value
      bpSend(0xEE, phrase)
      bpSend(0xEC, phrase)
      BP.status = "sent to TX + RX"
    end
    return 0
  end
  if BP.edit then
    if event == EVT_VIRTUAL_INC or event == EVT_VIRTUAL_INC_REPT then
      local i = BP.sidx + 1
      local c = string.sub(BP.value .. " ", i, i)
      local p = string.find(BP_CHARS, c, 1, true) or 1
      p = p + 1
      if p > #BP_CHARS then p = 1 end
      local left = string.sub(BP.value, 1, i - 1)
      local right = string.sub(BP.value, i + 1)
      BP.value = left .. string.sub(BP_CHARS, p, p) .. right
    elseif event == EVT_VIRTUAL_DEC or event == EVT_VIRTUAL_DEC_REPT then
      local i = BP.sidx + 1
      local c = string.sub(BP.value .. " ", i, i)
      local p = string.find(BP_CHARS, c, 1, true) or 1
      p = p - 1
      if p < 1 then p = #BP_CHARS end
      local left = string.sub(BP.value, 1, i - 1)
      local right = string.sub(BP.value, i + 1)
      BP.value = left .. string.sub(BP_CHARS, p, p) .. right
    elseif event == EVT_VIRTUAL_NEXT then
      if BP.sidx < BP_MAX - 1 then
        BP.sidx = BP.sidx + 1
        if BP.sidx >= #BP.value then BP.value = BP.value .. "a" end
      end
    elseif event == EVT_VIRTUAL_PREV then
      if BP.sidx > 0 then BP.sidx = BP.sidx - 1 end
    end
  end
  return 0
end

----------------------------------------------------------------------
-- Native mLRS MBridge page (CRSF 129/130). Opened from [mLRS Setup].
----------------------------------------------------------------------
MB.active = false
MB.page = 0
MB.line = 1
MB.edit = false
MB.sidx = 0
MB.tlast = 0
MB.save_t = 0
MB.item_tx = nil
MB.item_rx = nil
MB.info = nil
MB.plist = nil
MB.expect = 0
MB.current = -1
MB.errors = 0
MB.complete = false
MB.loading = true
MB.connected = false
MB.rows = {}

local MBCMD = {
  REQUEST_INFO = 3,
  DEVICE_ITEM_TX = 4,
  DEVICE_ITEM_RX = 5,
  PARAM_ITEM = 7,
  PARAM_ITEM2 = 8,
  PARAM_ITEM3_4 = 9,
  REQUEST_CMD = 10,
  INFO = 11,
  PARAM_SET = 12,
  PARAM_STORE = 13,
}
local MBTYPE = { UINT8 = 0, INT8 = 1, LIST = 4, STR6 = 5 }
local MBSTX = 0xA0
local MBLEN = { [2]=22, [3]=0, [4]=24, [5]=24, [7]=24, [8]=24, [9]=24, [10]=18, [11]=24, [12]=7, [13]=0 }

local function mb_str(p, pos, len)
  local s = ""
  for i = 0, len-1 do
    local b = p[pos+i]
    if b == nil or b == 0 then break end
    s = s .. string.char(b)
  end
  return s
end

local function mb_u16(p, pos)
  return (p[pos] or 0) + (p[pos+1] or 0) * 256
end

local function mb_opts(p, pos, len)
  local str = mb_str(p, pos, len) .. ","
  local opt = {}
  for s in string.gmatch(str, "([^,]+)") do
    opt[#opt+1] = s
  end
  return opt
end

function MB.push(cmd, payload)
  local data = { 79, 87, cmd + MBSTX }
  local n = MBLEN[cmd] or 0
  for i = 1, n do data[#data+1] = 0 end
  payload = payload or {}
  for i = 1, #payload do data[3+i] = payload[i] end
  local ok = crossfireTelemetryPush(129, data)
  if not ok then
    MB.pending = { cmd = cmd, payload = payload }
  else
    MB.pending = nil
  end
  return ok
end

function MB.pop()
  local cmd, data = crossfireTelemetryPop()
  if cmd ~= 130 or data == nil or data[1] == nil then return nil end
  local command = data[1] - MBSTX
  local res = { cmd = command, payload = {} }
  for i = 2, #data do res.payload[i-2] = data[i] end
  return res
end

function MB.reset()
  MB.item_tx = nil
  MB.item_rx = nil
  MB.info = nil
  MB.plist = nil
  MB.expect = 0
  MB.current = -1
  MB.errors = 0
  MB.complete = false
  MB.loading = true
  MB.page = 0
  MB.line = 1
  MB.edit = false
  MB.rows = {}
  MB.pending = nil
end

function MB.open()
  MB.active = true
  MB.reset()
  MB.tlast = 0
  MB.save_t = getTime()
end

function MB.build_rows()
  MB.rows = {}
  if not MB.complete or MB.plist == nil then return end
  local prefix = (MB.page == 1) and "Tx" or "Rx"
  if MB.page == 0 then
    MB.rows[#MB.rows+1] = { kind = "cmd", name = "Edit Rx", act = "rx" }
    MB.rows[#MB.rows+1] = { kind = "cmd", name = "Save", act = "save" }
    MB.rows[#MB.rows+1] = { kind = "cmd", name = "Reload", act = "reload" }
    MB.rows[#MB.rows+1] = { kind = "cmd", name = "Back to ELRS", act = "back" }
    if MB.info ~= nil and (MB.info.hop_count or 0) > 0 then
      MB.rows[#MB.rows+1] = {
        kind = "info",
        name = string.format("Hop skip %d/%d", MB.info.hop_skip or 0,
                             MB.info.hop_count)
      }
    end
  else
    for pidx = 2, 255 do
      local p = MB.plist[pidx]
      if p == nil then break end
      if string.sub(p.name, 1, 2) == prefix and (p.allowed_mask or 0) > 0 then
        MB.rows[#MB.rows+1] = { kind = "p", pidx = pidx, name = string.sub(p.name, 4) }
      end
    end
    MB.rows[#MB.rows+1] = { kind = "cmd", name = "Back", act = "main" }
  end
  if MB.line > #MB.rows then MB.line = #MB.rows end
  if MB.line < 1 then MB.line = 1 end
end

function MB.pval(p)
  if p == nil then return "---" end
  if p.typ == MBTYPE.STR6 then return p.value or "" end
  if p.typ == MBTYPE.LIST then
    return (p.options and p.options[(p.value or 0)+1]) or "?"
  end
  return tostring(p.value or 0) .. (p.unit or "")
end

function MB.send_set(pidx)
  local p = MB.plist[pidx]
  if p == nil then return end
  if p.typ == MBTYPE.STR6 then
    local v = p.value or ""
    local cmd = { pidx }
    for i = 1, 6 do cmd[i+1] = string.byte(string.sub(v.."      ", i, i)) end
    MB.push(MBCMD.PARAM_SET, cmd)
  else
    MB.push(MBCMD.PARAM_SET, { pidx, bit32.band(p.value or 0, 255) })
  end
end

function MB.inc(p, dir)
  if p.typ == MBTYPE.STR6 then
    local chars = "abcdefghijklmnopqrstuvwxyz0123456789_#-."
    local v = p.value or "      "
    if #v < 6 then v = v .. string.rep(" ", 6-#v) end
    local i = (MB.sidx or 0) + 1
    local c = string.sub(v, i, i)
    local pos = string.find(chars, c, 1, true) or 1
    pos = pos + dir
    if pos < 1 then pos = #chars elseif pos > #chars then pos = 1 end
    p.value = string.sub(v, 1, i-1) .. string.sub(chars, pos, pos) .. string.sub(v, i+1)
    return
  end
  if p.typ == MBTYPE.LIST then
    local value = p.value or 0
    local max = p.max or 0
    local start = value
    local mask = p.allowed_mask or 65535
    if max < 0 then return end
    while true do
      value = value + dir
      if value < 0 then
        value = max
      elseif value > max then
        value = 0
      end
      if value == start then return end
      if bit32.btest(bit32.lshift(1, value), mask) then
        p.value = value
        return
      end
    end
  else
    local min = p.min or 0
    local max = p.max or 0
    local v = (p.value or 0) + dir
    if v < min then v = min end
    if v > max then v = max end
    p.value = v
  end
end

function MB.handle_cmd(cmd)
  if cmd.cmd == MBCMD.DEVICE_ITEM_TX then
    MB.item_tx = cmd
    MB.item_tx.name = mb_str(cmd.payload, 4, 20)
  elseif cmd.cmd == MBCMD.DEVICE_ITEM_RX then
    MB.item_rx = cmd
    MB.item_rx.name = mb_str(cmd.payload, 4, 20)
  elseif cmd.cmd == MBCMD.INFO then
    MB.info = cmd
    MB.info.rx_available = bit32.band(cmd.payload[5] or 0, 1)
    MB.info.param_num = cmd.payload[8] or 0
    local txb = cmd.payload[3] or 0
    local rxb = cmd.payload[4] or 0
    if txb > 127 then txb = txb - 256 end
    if rxb > 127 then rxb = rxb - 256 end
    MB.info.tx_dbm = txb
    MB.info.rx_dbm = rxb
    MB.info.hop_skip = cmd.payload[9] or 0
    MB.info.hop_count = cmd.payload[10] or 0
    MB.info.hop_mask = (cmd.payload[11] or 0)
      + (cmd.payload[12] or 0) * 256
      + (cmd.payload[13] or 0) * 65536
      + (cmd.payload[14] or 0) * 16777216
    MB.info.rate = cmd.payload[15] or 0
    MB.info.band = cmd.payload[16] or 0
    MB.info.ul_plen = cmd.payload[17] or 0
    MB.info.dl_plen = cmd.payload[18] or 0
    -- Rebuild only when hop/skip text changes. Payload lengths change every
    -- packet and were rebuilding the page ~3 Hz, which hiccups CRSF.
    local hop_key = string.format("%d/%d/%d", MB.info.hop_skip or 0,
                                  MB.info.hop_count or 0, MB.info.hop_mask or 0)
    if MB.complete and MB.page == 0 and hop_key ~= MB.hop_key then
      MB.hop_key = hop_key
      MB.build_rows()
    end
  elseif cmd.cmd == MBCMD.PARAM_ITEM then
    local index = cmd.payload[0]
    if MB.complete and index ~= 255 and MB.plist and MB.plist[index] then
      local p = MB.plist[index]
      if p.typ == MBTYPE.STR6 then
        p.value = mb_str(cmd.payload, 18, 6)
      elseif p.typ == MBTYPE.INT8 then
        local v = cmd.payload[18] or 0
        if v >= 128 then v = v - 256 end
        p.value = v
      else
        p.value = cmd.payload[18]
      end
    elseif index ~= MB.expect and index ~= 255 then
      MB.errors = MB.errors + 1
    end
    if not MB.complete then
      MB.current = index
      MB.expect = index + 1
    end
    if MB.plist == nil then
      MB.errors = MB.errors + 1
    elseif index == 255 then
      MB.complete = (MB.errors == 0)
      MB.loading = false
      MB.build_rows()
    elseif not MB.complete and index < 128 then
      MB.plist[index] = cmd
      cmd.typ = cmd.payload[1]
      cmd.name = mb_str(cmd.payload, 2, 16)
      if cmd.typ == MBTYPE.STR6 then
        cmd.value = mb_str(cmd.payload, 18, 6)
      elseif cmd.typ == MBTYPE.INT8 then
        local v = cmd.payload[18] or 0
        if v >= 128 then v = v - 256 end
        cmd.value = v
      else
        cmd.value = cmd.payload[18]
      end
      cmd.options = {}
      cmd.allowed_mask = 65536
      cmd.min = 0
      cmd.max = 0
      cmd.unit = ""
    end
  elseif cmd.cmd == MBCMD.PARAM_ITEM2 then
    local index = cmd.payload[0]
    local p = MB.plist and MB.plist[index]
    if p == nil then
      MB.errors = MB.errors + 1
    else
      local need3 = false
      if p.typ == MBTYPE.LIST then
        p.allowed_mask = mb_u16(cmd.payload, 1)
        p.options = mb_opts(cmd.payload, 3, 21)
        p.item2 = cmd.payload
        p.max = #p.options - 1
        if #mb_str(cmd.payload, 3, 21) == 21 then need3 = true end
      elseif p.typ == MBTYPE.UINT8 then
        p.min = cmd.payload[1] or 0
        p.max = cmd.payload[3] or 0
        p.unit = mb_str(cmd.payload, 7, 6)
      elseif p.typ == MBTYPE.INT8 or p.typ < MBTYPE.LIST then
        local function i8(v)
          v = v or 0
          if v >= 128 then v = v - 256 end
          return v
        end
        p.min = i8(cmd.payload[1])
        p.max = i8(cmd.payload[3])
        p.unit = mb_str(cmd.payload, 7, 6)
      end
      if not need3 and not MB.complete then
        MB.push(MBCMD.REQUEST_CMD, { MBCMD.PARAM_ITEM, MB.expect })
      end
    end
  elseif cmd.cmd == MBCMD.PARAM_ITEM3_4 then
    local index = cmd.payload[0]
    local is4 = false
    if index >= 128 then index = index - 128; is4 = true end
    local p = MB.plist and MB.plist[index]
    if p == nil or p.item2 == nil then
      MB.errors = MB.errors + 1
    else
      local need4 = false
      if not is4 then
        p.item3 = cmd.payload
        local s = p.item2
        for i = 1, 23 do s[23+i] = cmd.payload[i] end
        p.options = mb_opts(s, 3, 21+23)
        if #mb_str(cmd.payload, 1, 23) == 23 then need4 = true end
      else
        local s = p.item2
        local s3 = p.item3
        for i = 1, 23 do s[23+i] = s3[i]; s[23+23+i] = cmd.payload[i] end
        p.options = mb_opts(s, 3, 21+23+23)
      end
      p.max = #p.options - 1
      if not need4 and not MB.complete then
        MB.push(MBCMD.REQUEST_CMD, { MBCMD.PARAM_ITEM, MB.expect })
      end
    end
  end
end

function MB.poll()
  local t = getTime()
  if MB.pending ~= nil then
    MB.push(MB.pending.cmd, MB.pending.payload)
  end
  if t - MB.tlast > 33 then
    MB.tlast = t
    if t < MB.save_t + 300 then
      -- dead time after Save
    elseif MB.item_tx == nil then
      MB.push(MBCMD.REQUEST_INFO, {})
      MB.expect = 0
      MB.current = -1
      MB.errors = 0
      MB.complete = false
    elseif MB.plist == nil then
      if MB.info ~= nil then
        MB.plist = {}
        MB.push(MBCMD.REQUEST_CMD, { MBCMD.PARAM_ITEM, MB.expect })
      end
    end
  end
  for _ = 1, 24 do
    local cmd, data = crossfireTelemetryPop()
    if cmd == nil then break end
    if cmd == 130 and data ~= nil and data[1] ~= nil then
      local res = { cmd = data[1] - MBSTX, payload = {} }
      for i = 2, #data do res.payload[i-2] = data[i] end
      MB.handle_cmd(res)
    end
  end
end

function MB.draw()
  lcd.clear()
  local y0 = 0
  local th = titleH
  local hh = tlmH
  if lcd.RGB ~= nil then
    if th == 0 then th = 24 end
    if hh == 0 then hh = 18 end
    lcd.setColor(CUSTOM_COLOR, lcd.RGB(0x9f, 0xc7, 0x6f))
    lcd.drawFilledRectangle(0, 0, LCD_W, th, CUSTOM_COLOR)
    lcd.setColor(CUSTOM_COLOR, lcd.RGB(0, 0, 0))
    lcd.drawText(4, 4, "mLRS Setup", CUSTOM_COLOR + BOLD)
    local sub = (MB.page == 2 and "Edit Rx") or "Main"
    lcd.drawText(LCD_W-4, 4, sub, CUSTOM_COLOR + RIGHT)
    drawTlmStrip(th, hh)
    y0 = th + hh + 4
  else
    if th == 0 then th = 9 end
    if hh == 0 then hh = 9 end
    lcd.drawFilledRectangle(0, 0, LCD_W, th, GREY_DEFAULT)
    lcd.drawText(0, 1, "mLRS Setup", INVERS)
    drawTlmStrip(th, hh)
    y0 = th + hh + 1
  end

  local rssi = getRSSI()
  MB.connected = (rssi ~= nil and rssi ~= 0)
  local info = "TX " .. ((MB.item_tx and MB.item_tx.name) or "---")
  if MB.connected then
    info = info .. "  RX " .. ((MB.item_rx and MB.item_rx.name) or "---")
  else
    info = info .. "  RX not connected"
  end
  lcd.drawText(4, y0, info, SMLSIZE)
  y0 = y0 + ((lcd.RGB ~= nil) and 18 or 10)

  if MB.loading then
    local n = MB.current
    if n < 0 then n = 0 end
    lcd.drawText(4, y0 + 8, "Loading mLRS params... ("..tostring(n)..")", BLINK)
    lcd.drawText(4, y0 + 28, "Air Protocol must be mLRS", SMLSIZE)
    return
  end

  local dy = (lcd.RGB ~= nil) and 20 or 9
  local maxn = math.floor((LCD_H - y0 - 4) / dy)
  local top = 0
  if MB.line > maxn then top = MB.line - maxn end
  for i = 1, maxn do
    local row = MB.rows[top + i]
    if row == nil then break end
    local y = y0 + (i-1)*dy
    local attr = 0
    if (top + i) == MB.line then
      attr = INVERS
      if MB.edit then attr = attr + BLINK end
    end
    lcd.drawText(4, y, row.name, attr)
    if row.kind == "p" then
      local p = MB.plist[row.pidx]
      local val = MB.pval(p)
      if MB.edit and (top + i) == MB.line and p and p.typ == MBTYPE.STR6 then
        val = ""
        local s = p.value or ""
        for ci = 1, 6 do
          local ch = string.sub(s, ci, ci)
          if ch == "" then ch = " " end
          if (ci-1) == MB.sidx then
            val = val .. "[" .. ch .. "]"
          else
            val = val .. ch
          end
        end
      end
      lcd.drawText(LCD_W/2, y, val, attr)
    end
  end
end

function MB.ensure_list(p)
  if p == nil or p.typ ~= MBTYPE.LIST then return end
  if p.options ~= nil and #p.options >= 2 then
    p.max = #p.options - 1
    return
  end
  if (p.max or 0) <= 1 then
    p.options = {"Off", "On"}
    p.max = 1
    if (p.allowed_mask or 0) == 0 or p.allowed_mask == 65536 then
      p.allowed_mask = 3
    end
  end
end

function MB.on_enter()
  local row = MB.rows[MB.line]
  if row == nil then return end
  if row.kind == "cmd" then
    if row.act == "rx" then MB.page = 2; MB.line = 1; MB.build_rows()
    elseif row.act == "save" then
      MB.push(MBCMD.PARAM_STORE, {})
      MB.save_t = getTime()
      MB.reset()
    elseif row.act == "reload" then MB.reset()
    elseif row.act == "back" then MB.active = false
    elseif row.act == "main" then MB.page = 0; MB.line = 1; MB.build_rows()
    end
    return
  end
  local p = MB.plist[row.pidx]
  if p == nil then return end
  MB.ensure_list(p)
  if p.typ == MBTYPE.LIST then
    local mask = p.allowed_mask or 65535
    if mask ~= 65536 then
      local bits = 0
      local m = mask
      while m > 0 and bits < 2 do
        bits = bits + bit32.band(m, 1)
        m = bit32.rshift(m, 1)
      end
      if bits <= 1 then return end
    end
  end
  MB.edit = true
  MB.sidx = 0
end

function MB.run(event)
  MB.poll()
  if event == EVT_VIRTUAL_EXIT then
    if MB.edit then
      local row = MB.rows[MB.line]
      local p = row and MB.plist[row.pidx]
      if p and p.typ == MBTYPE.STR6 then MB.send_set(row.pidx) end
      MB.edit = false
    elseif MB.page ~= 0 then
      MB.page = 0
      MB.line = 1
      MB.build_rows()
    else
      MB.active = false
    end
  elseif event == EVT_VIRTUAL_ENTER then
    if MB.edit then
      local row = MB.rows[MB.line]
      local p = row and MB.plist[row.pidx]
      if p and p.typ == MBTYPE.STR6 then
        MB.sidx = MB.sidx + 1
        if MB.sidx >= 6 then
          MB.send_set(row.pidx)
          MB.edit = false
          MB.sidx = 0
        end
      else
        if row and row.kind == "p" then MB.send_set(row.pidx) end
        MB.edit = false
      end
    else
      MB.on_enter()
    end
  elseif event == EVT_VIRTUAL_NEXT then
    if MB.edit then
      local row = MB.rows[MB.line]
      local p = row and MB.plist[row.pidx]
      if p then
        MB.ensure_list(p)
        MB.inc(p, 1)
        if p.typ ~= MBTYPE.STR6 then MB.send_set(row.pidx) end
      end
    else
      MB.line = math.min(MB.line + 1, math.max(1, #MB.rows))
    end
  elseif event == EVT_VIRTUAL_PREV then
    if MB.edit then
      local row = MB.rows[MB.line]
      local p = row and MB.plist[row.pidx]
      if p then
        MB.ensure_list(p)
        MB.inc(p, -1)
        if p.typ ~= MBTYPE.STR6 then MB.send_set(row.pidx) end
      end
    else
      MB.line = math.max(MB.line - 1, 1)
    end
  end
  MB.draw()
  return 0
end

-- Init
local function init()
  setLCDvar()
  setMock()
  setLCDvar = nil
  setMock = nil
end

-- Main
local function run(event, touchState)
  if event == nil then return 2 end
  if checkCrsfModule then return checkCrsfModule() end

  event = (touch2evt and touch2evt(event, touchState)) or event
  if BP.active then
    return BP.run(event)
  end
  if MB.active then
    return MB.run(event)
  end
  -- If ENTER pressed, skip any pushing this loop to reserve queue for the save command
  local forceRedraw = refreshNext(event == EVT_VIRTUAL_ENTER)

  if fieldPopup ~= nil then
    runPopupPage(event)
  elseif event ~= 0 or forceRedraw or edit then
    runDevicePage(event)
  end

  return exitscript
end

return { init=init, run=run }
