function [ ] = setDefaultGains_XSeries( familyName )
%SETDEFAULTGAINS_XSERIES puts the 'factory default' gains on a module.
%
% [] = setXSeriesDefaultGains( familyName )
%
% 'familyName' is a string for the family of all the modules that you
% want to set.  This function will search for various modules that have
% this family name and set the appropriate gains for each type based on
% their hardware type.
%
% This function assumes that all the XML files for the default gains are
% located in the same directory as this function.  These files can be
% downloaded from:
% http://docs.hebi.us/#downloadable-default-gains
%
% Dave Rollinson
% Sep 2017

if nargin<1
    familyName = '*';
end

    % Get absolute filepath for finding gains .xml files
	localDir = fileparts(mfilename('fullpath'));
    
    ledPauseTime = .1;

    fprintf('Clearing Module List...');
    HebiLookup.clearModuleList();
    pause(1.0);
    fprintf('DONE!\n');
  
    allModules = HebiLookup.newGroupFromFamily(familyName);

    % Make cell arrays, even if only a single module
    allNames = allModules.getInfo.name;
    allMACs = allModules.getInfo.macAddress;
    allTypes = allModules.getInfo.mechanicalType;
    
    % List of all types to iterate thru
    mechTypes = { 'X5-1', ...
                  'X5-4', ...
                  'X5-9', ...
                  'X8-3', ...
                  'X8-9', ...
                  'X8-16' };

    for i=1:length(mechTypes)
        
        fprintf(['Setting ' mechTypes{i} ' Module Gains...']);
        
        gainGroupMACs = allMACs( strcmp( allTypes, mechTypes{i} ) );
        
        if ~isempty(gainGroupMACs)
            
            % Make a group of a certain type.  
            gainGroup = HebiLookup.newGroupFromMacs(gainGroupMACs);
            numModules = gainGroup.getNumModules();
            
            % Load XML files and set control strategies 2, 4, 3.
            % Strategy 3 is set last because it is the default.
            for strategy = 3; %[2 4 3]
                gains = HebiUtils.loadGains( [localDir '/' mechTypes{i} ...
                    '_STRATEGY' num2str(strategy)] );
                gains = expandGainGroup( gains, numModules );
                gainGroup.send('gains', gains, 'led', 'y','persist',true);
                pause(ledPauseTime);
                gainGroup.send('led', []);
            end
            
%             % Persist everything so that the modules retain the values
%             % after reboot.
%             gainGroup.send('persist',true);
            
            fprintf('DONE!\n');
        else
            fprintf(['No ' mechTypes{i} ' Modules Found!\n']);
        end
    end

end

% Helper function to expand out the single values of the gains in the XML
% files to whatever the group size is for setting
function [gains] = expandGainGroup( gains, numModules )
    gainFields = fields(gains);
    for i=2:length(gainFields)
        gains.(gainFields{i}) = gains.(gainFields{i}) * ones(1,numModules);
    end
end