-- This is an example of using Lua statemachines
function onEntry ()
   print "--> enter Pa state"
end

function onActive ()
   print " Pa"
   return 3
end

function onExit ()
   print "<-- exit Pa state"
   print ""
end
