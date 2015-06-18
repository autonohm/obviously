-- This is an example of using Lua statemachines
function doEntry ()
   print "--> enter Pi state"
end

function doActive ()
   print " Pi"
   return 0
end

function doExit ()
   print "<-- exit Pi state"
   print ""
end
