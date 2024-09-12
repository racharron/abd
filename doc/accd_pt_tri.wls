(*
A = Table[a[i, j], {i, 3}, {j, 3}]
T = Table[t[i], {i, 3}]
dA = Table[da[i, j], {i, 3}, {j, 3}]
dT = Table[dt[i], {i, 3}]

B = Table[b[i, j], {i, 3}, {j, 3}]
U = Table[u[i], {i, 3}]
dB = Table[db[i, j], {i, 3}, {j, 3}]
dU = Table[du[i], {i, 3}]

pt0 = Table[vp[i], {i, 3}]
ta0 = Table[va[i], {i, 3}]
tb0 = Table[vb[i], {i, 3}]
tc0 = Table[vc[i], {i, 3}]
*)

Share[]

Coplanar[x1_, x2_, x3_, x4_] := 0 == Dot[Cross[x2-x1, x4-x1], x3-x1]

point := dT * tau + pt0(* (dA . pt0 + dT) * tau + A . pt0 + T *)

tri[v_] := dU * tau + v (* (dB . v + dU) * tau + B . v + U *)
ta = tri[ta0]
tb = tri[tb0]
tc = tri[tc0]

ab = tb - ta
bc = tc - tb
ca = ta - tc
ap = point - ta
bp = point - tb
cp = point - tc

orth = Cross[ab, bc]

inab = Dot[orth, Cross[ab, ap]]
inbc = Dot[orth, Cross[bc, bp]]
inca = Dot[orth, Cross[ca, cp]]


vectors = Fold[{a,b} |-> a && b, Map[v |-> Element[v, Vectors[3, Reals]], {dT, dU, pt0, ta0, tb0, tc0}]]

(*
matrices = Fold[{a,b} |-> a && b, Map[m |-> Element[m, Matrices[{3, 3}, Reals]], {dA, dB}]]
*)

(*
{solge, solle} = Parallelize[{
    Minimize[
        {
            tau, 
            tau >= 0,
            Coplanar[point, ta, tb, tc],
            inab >= 0,
            inbc >= 0,
            inca >= 0,
			vectors,
			matrices
        },
        tau,
        Reals
    ],
    Minimize[
        {
            tau, 
            tau >= 0,
            Coplanar[point, ta, tb, tc],
            inab <= 0,
            inbc <= 0,
            inca <= 0,
			vectors,
			matrices
        },
        tau,
        Reals
    ]
}]
*)

(*
		Element[T, Vectors[3, Reals]],
		Element[dT, Vectors[3, Reals]],
		Element[U, Vectors[3, Reals]],
		Element[dU, Vectors[3, Reals]],
		Element[pt0, Vectors[3, Reals]],
		Element[ta0, Vectors[3, Reals]],
		Element[tb0, Vectors[3, Reals]],
		Element[tc0, Vectors[3, Reals]],
*)
(*
compiled = Compile[
	{},
	Minimize[
		{
			tau, 
			tau >= 0
			(* && tau <= delta
			&& delta > 0 *)
			&& Coplanar[point, ta, tb, tc]
			&& inab >= 0
			&& inbc >= 0
			&& inca >= 0
			&& vectors
			(* && matrices *)
			&& Element[tau, Reals]
			(* && Element[delta, Reals] *)
		},
		tau
		(*
		Element[T, Vectors[3, Reals]],
		Element[dT, Vectors[3, Reals]],
		Element[U, Vectors[3, Reals]],
		Element[dU, Vectors[3, Reals]],
		Element[pt0, Vectors[3, Reals]],
		Element[ta0, Vectors[3, Reals]],
		Element[tb0, Vectors[3, Reals]],
		Element[tc0, Vectors[3, Reals]],
		Element[A, Matrices[{3, 3}, Reals]],
		Element[dA, Matrices[{3, 3}, Reals]],
		Element[B, Matrices[{3, 3}, Reals]],
		Element[dB, Matrices[{3, 3}, Reals]],
		Element[tau, Reals]
		*)
	]
]
*)

$Assumptions = Element[dT | dU | pt0 | ta0 | tb0 | tc0, Vectors[3, Reals]] && Element[tau, Reals]

(*
Print["Compiled solver"]
*)

intersection = Solve[
	tau >= 0
	&& Coplanar[point, ta, tb, tc]
	&& inab >= 0
	&& inbc >= 0
	&& inca >= 0
	&& $Assumptions,
	tau
]

Export["tri_point_intersection.txt", intersection]

(*
Print["GE: ", solge]
Print[]
Print["LE: ", solle]
*)

Print["Triangle-Point Intersection:"]
Print[intersection]
Print[]


(*
Print["Triangle-Point Affine Collision:"]
Print[tpsol]
*)