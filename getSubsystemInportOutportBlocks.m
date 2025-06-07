function [inportBlockHandles, outportBlockHandles] = getSubsystemInportOutportBlocks(subsystemHandle)
    % Initialize output arguments as empty cell arrays
    inportBlockHandles = {};
    outportBlockHandles = {};

    % Error checking for the input handle
    isValidHandle = ishandle(subsystemHandle);
    isSubSystem = false;

    if isValidHandle
        try
            if strcmp(get_param(subsystemHandle, 'Type'), 'block') && ...
               strcmp(get_param(subsystemHandle, 'BlockType'), 'SubSystem')
                isSubSystem = true;
            end
        catch E
            isValidHandle = false;
        end
    end

    if ~isValidHandle || ~isSubSystem
        disp('Input is not a valid Subsystem block.');
        return;
    end

    % Find Inport blocks directly within the subsystem
    foundInports = find_system(subsystemHandle, ...
                               'SearchDepth', 1, ...
                               'LookUnderMasks', 'all', ...
                               'FollowLinks', 'on', ...
                               'BlockType', 'Inport');

    if ~isempty(foundInports)
        inportBlockHandles = num2cell(foundInports);
    else
        inportBlockHandles = {};
    end

    % Find Outport blocks directly within the subsystem
    foundOutports = find_system(subsystemHandle, ...
                                'SearchDepth', 1, ...
                                'LookUnderMasks', 'all', ...
                                'FollowLinks', 'on', ...
                                'BlockType', 'Outport');

    if ~isempty(foundOutports)
        outportBlockHandles = num2cell(foundOutports);
    else
        outportBlockHandles = {}; % Ensure it's an empty cell if nothing found
    end
end
