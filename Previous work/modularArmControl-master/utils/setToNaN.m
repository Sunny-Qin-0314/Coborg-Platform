cmd.position = NaN(1,n);
cmd.velocity = NaN(1,n);
cmd.effort = NaN(1,n);
                try
            robot.set(cmd);
        catch err
            disp(err.message);
        end