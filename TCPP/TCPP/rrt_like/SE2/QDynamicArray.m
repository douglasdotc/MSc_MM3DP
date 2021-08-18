classdef QDynamicArray < handle
    %JARRAY is a normal dynamic array implementation

    properties
        array
        idx
        sz
        grow_ratio = 1.5;
    end

    methods

        function self = QDynamicArray(start_array)
            self.array = start_array;
            self.sz = length(self.array);
            self.idx = self.sz;

        end

        function append(self, elems)

            for ii = 1:length(elems)
                self.append_1(elems(ii))
            end

        end

        function arr = get(self, varargin)

            arr = self.array(1:self.idx);            

        end

    end

    methods (Access = private)

        function append_1(self, single_element)

            if self.idx + 1 < self.sz
                self.array(self.idx + 1) = single_element;
                self.idx = self.idx + 1;
            else
                self.grow();
                self.append_1(single_element);
            end

        end

        function grow(self)
            l = ceil(self.sz * (self.grow_ratio - 1));
            self.array = cat(1, self.array, self.new_arr(l));
            self.sz = length(self.array);
        end

        function ar = new_arr(self, sz)
            ar = arrayfun(@(~) QSE2T([-1, -1, -1, -1]), zeros(1, sz))';
        end

        % function shrink(self)
        %     max_ind = ceil(self.sz * self.shrink_ratio);
        %     self.array = self.array(1:max_ind);
        %     self.sz = length(self.array);

        % end

    end

end
