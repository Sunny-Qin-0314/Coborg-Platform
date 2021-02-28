% setToZero

  % send to hardware
            cmd.effort = zeros(1,n);
                        cmd.position = NaN(1,n);

             try
            robot.set(cmd);
        catch err
            disp(err.message);
        end
            
      