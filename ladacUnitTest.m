% ladacUnitTest implements tests for LADAC
%
%   These functions define test cases with well known results.
%   This unit test should be run every time a file has been modified,
%   to prove that results are still as expected.
%   If any changes has been made to the function this test
%   script can be used to find unintended errors.
%
%   example calls:
%   rt = table(runtests('ladacUnitTest'))
%   rt = table(runtests('ladacUnitTest','OutputDetail',0))

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
%
%   Copyright (C) 2020-2022 Fabian Guecker
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% Call all subfunction tests in this file
function tests = ladacUnitTest
tests = functiontests(localfunctions);
end

%% ------------------------------------------------------------------------
function Test_all_library_links_resolved(testCase)

parent_dir = fileparts(mfilename('fullpath'));
unresolved_links = validateLibraryLinks(parent_dir, false);

verifyEqual(testCase, unresolved_links, {});

end





