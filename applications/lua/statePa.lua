-- This is an example of using Lua statemachines
function doEntry ()
   print "--> enter Pa state"
end

function doActive ()
   print " Pa"
   return 0
end

function doExit ()
   print "<-- exit Pa state"
   print ""
end
