%% disclaimer
% Copyright 2019 Gianluca Frison, Dimitris Kouzoupis, Robin Verschueren,
% Andrea Zanelli, Niels van Duijkeren, Jonathan Frey, Tommaso Sartor,
% Branimir Novoselnik, Rien Quirynen, Rezart Qelibari, Dang Doan,
% Jonas Koenemann, Yutao Chen, Tobias Schöls, Jonas Schlagenhauf, Moritz Diehl
%
% This file is part of acados.
%
% The 2-Clause BSD License
%
% Redistribution and use in source and binary forms, with or without
% modification, are permitted provided that the following conditions are met:
%
% 1. Redistributions of source code must retain the above copyright notice,
% this list of conditions and the following disclaimer.
%
% 2. Redistributions in binary form must reproduce the above copyright notice,
% this list of conditions and the following disclaimer in the documentation
% and/or other materials provided with the distribution.
%
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
% AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
% IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
% ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
% LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
% CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
% SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
% INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
% CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
% ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
% POSSIBILITY OF SUCH DAMAGE.;

%% BOF
function model = ocp_model_NMPC_v8_LSTM()
% import casadi.*
init_modvehicle_IPG;    %Init Vehicle parameters
init_MPC_LSTM;     % Init constants MPC and Options P,T, etc


%% define the symbolic variables of the plant
ocp_S02_DefACADOSVarSpace_ac_v8_LSTM;

%% define ode rhs in explicit form (22 equations)
ocp_S04_SetupNonlinearStateSpaceDynamics_ac_v8_LSTM;

%% generate casadi C functions
nx = options.n_states;
nu = options.n_controls;
np = options.n_parameter;

%% populate structure
model.nx = nx;
model.nu = nu;
model.np = np;
model.sym_x = x;
% model.sym_xdot = dx;
model.sym_u = u;
model.sym_p = p;
% model.expr_f_expl = f_expl;
% model.expr_f_impl = f_impl;
model.expr_phi = expr_phi;
model.expr_h = h;
model.expr_h_e = h_e;
model.expr_g = g;
model.expr_g_e = g_e;
model.expr_y = y;
model.expr_y_e = y_e;

