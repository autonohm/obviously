-- This is an example of using Lua statemachines
_agentID = -1

function onLoad (agentID)
   print "### load Pi state script"
   _agentID = agentID
end

function onEntry ()
   print "--> enter Pi state"
end

function onActive ()
   print "    activated Pi state"
   stateID = 2;
   transitionToPersistantState(_agentID, stateID);
   return 0
end

function onExit ()
   print "<-- exit Pi state"
   print ""
end
