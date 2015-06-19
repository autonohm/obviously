-- This is an example of using Lua statemachines
function onEntry ()
   print "--> enter Pi state"
end

function onActive ()
   print " Pi"
   return 2
end

function onExit ()
   print "<-- exit Pi state"
   print ""
end
