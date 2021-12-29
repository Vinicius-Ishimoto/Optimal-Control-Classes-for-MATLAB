function out = subsref(in,S)

for w=1:numel(S);
switch S(w).type
    case '()'
            out = in(S(w).subs{:});
    case '.'
        try
            out = eval(['in.',S(w).subs]);
        catch
            error('Not defined subscript.')
        end
    case '{}'
        out = in{S(w).subs{:}};
end
end

end