-- This is an example of using Lua statemachines
_agentID = -1

function onLoad (agentID)
   print "### load Pa state script"
   _agentID = agentID
end

function onEntry ()
   print "--> enter Pa state"
end

function onActive ()
   print "    activated Pa state"
   stateID = 3;
   transitionToPersistantState(_agentID, stateID);
   return 0
end

function onExit ()
   print "<-- exit Pa state"
   print ""
end
