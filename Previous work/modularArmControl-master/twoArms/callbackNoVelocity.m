function [cmdOut ] = callbackNoVelocity(time, fbk, cmd)

cmdOut = cmd;
cmdOut.velocity = nan(size(cmd.velocity));

end