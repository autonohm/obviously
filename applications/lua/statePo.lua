-- This is an example of using Lua statemachines
function onEntry ()
   print "--> enter Po state"
end

function onActive ()
   print " Po"
   return 1
end

function onExit ()
   print "<-- exit Po state"
   print ""
end
