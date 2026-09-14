function equivalent = equivalentVehicle(vehicle)
%EQUIVALENTVEHICLE Suppress left/right transfer for a Level 1 axle model.
% The common four-contact slip solver remains the single source of tire,
% force-balance and map physics. Very large virtual tracks make each axle's
% left/right loads equal without changing its total normal load.

equivalent = vehicle;
equivalent.modelOptions.lateralLoadTransferScale = 0;
end
