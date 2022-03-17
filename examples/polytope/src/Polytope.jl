module Polytope

using BenchmarkTools
using GeometryBasics
using JLD2
using LinearAlgebra
using MeshCat
using Meshing
using Pkg
using Plots
using Random
using StaticArrays

global OSF_PATH = joinpath("/home/simon/research/repos/osf-pytorch")
# include("setup.jl")
using PyCall

include("collider.jl")
include("utils.jl")
include("visualize.jl")
end # module
