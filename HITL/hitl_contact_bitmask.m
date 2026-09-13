function mask = hitl_contact_bitmask(active)
%HITL_CONTACT_BITMASK Encode six permanent ground contacts into TD_CNTCT.

active = logical(active(:));
mask = uint8(0);
for k = 1:min(numel(active), 6)
    if active(k)
        mask = bitor(mask, bitshift(uint8(1), k - 1));
    end
end
end
