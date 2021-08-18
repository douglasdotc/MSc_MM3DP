classdef QOrderedArray < handle
    %JARRAY is a normal dynamic array implementation

    properties
        array
        arrval
        idx
        sz
        grow_ratio = 1.5;
    end

    methods

        function self = QOrderedArray(start_array)

            qar = QSE2T.qmat(start_array);
            arrv = qar(:, 4);

            [arrv, I] = sort(arrv);
            self.arrval = arrv;
            self.array = start_array(I);
            self.sz = length(self.array);
            self.idx = self.sz;
        end

        function append(self, elems)

            for ii = 1:length(elems)
                self.append_1(elems(ii))
            end

        end

        function [arr, vals] = get(self, varargin)

            if isempty(varargin)
                arr = self.array(1:self.idx);
                vals = self.arrval(1:self.idx);
            else
                val = varargin{1};
                rangee = varargin{2};
                %%TODO I could even not return things t>0 val
                minv = max(val - rangee, 0);
                maxv = min(val + rangee, 1);
                inds = self.arrval >= minv & self.arrval <= maxv & self.arrval >= 0;
                arr = self.array(inds);
                vals = self.arrval(inds);

            end

        end

        function [node, maxt] = getmax(self)
            maxt = self.arrval(self.idx);
            node = self.array(self.idx);
        end

    end

    methods (Access = private)

        function append_1(self, single_element)

            idx_ = find(single_element.q(4) < self.arrval, 1);

            if isempty(idx_)
                idx_ = self.idx + 1;
            end

            self.insert_1(single_element, idx_);

        end

        function insert_1(self, single_element, idx)
            assert(idx <= self.idx + 1)

            if self.idx + 1 < self.sz

                self.array((idx + 2):self.sz + 1) = self.array((idx + 1):self.sz);
                self.array(idx + 1) = single_element;

                self.arrval((idx + 2):self.sz + 1) = self.arrval((idx + 1):self.sz);
                self.arrval(idx + 1) = single_element.q(4);

                self.idx = self.idx + 1;
            else
                self.grow();
                self.insert_1(single_element, idx)
            end

        end

        function grow(self)
            l = ceil(self.sz * (self.grow_ratio - 1));
            self.array = cat(1, self.array, self.new_arr(l));
            self.arrval = cat(1, self.arrval, -1 * ones(l, 1));
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
