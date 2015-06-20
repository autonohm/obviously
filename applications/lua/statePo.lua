-- This is an example of using Lua statemachines
_agentID = -1

function onLoad (agentID)
   print "### load Po state script"
   _agentID = agentID
end

function onEntry ()
   print "--> enter Po state"
end

function onActive ()
   print "    activated Po state"
   stateID = 1;
   transitionToPersistantState(_agentID, stateID);
   return 0
end

function onExit ()
   print "<-- exit Po state"
   print ""
end
