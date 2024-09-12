Norm2[-v_] := Norm[v]

tA0 = {a0x, a0y, a0z};
tA1 = {a1x, a1y, a1z};
tB0 = {b0x, b0y, b0z};
tB1 = {b1x, b1y, b1z};
tP = {px, py, pz};

$Assumptions = (A0 | A1 | B0 | B1) \[Element] Vectors[3, Reals] &&
	(t1 | t2 | s) \[Element] Reals &&
	Fold[#1 | #2 &, Join[tA0, tA1, tB0, tB1, tP]] \[Element] Reals;

{t1`numer, t1`denom} = Simplify /@ Block[
	{
		Compile`$20, Compile`$36, Compile`$23, Compile`$35, Compile`$62, Compile`$44,
		Compile`$13, Compile`$98, Compile`$133, Compile`$140, Compile`$141, Compile`$142, Compile`$177, Compile`$155,
		Compile`$156, Compile`$157, Compile`$161, Compile`$163, Compile`$183, Compile`$184, Compile`$37, Compile`$63,
		Compile`$120, Compile`$134, Compile`$139, Compile`$145, Compile`$170, Compile`$178, Compile`$146, Compile`$182,
		Compile`$185, Compile`$186, Compile`$187, Compile`$196
	},
	Compile`$20 = b0x^2; Compile`$36 = b0y^2;
	Compile`$23 = a1z^2; Compile`$35 = a0x^2; Compile`$62 = b0z^2; Compile`$44 = a1x^2; Compile`$13 = a1y^2;
	Compile`$98 = b1x^2; Compile`$133 = b1y^2; Compile`$140 = -2*b0x*b1x; Compile`$141 = -2*b0y*b1y;
	Compile`$142 = Compile`$20 + Compile`$36 + Compile`$140 + Compile`$98 + Compile`$141 + Compile`$133;
	Compile`$177 = b1z^2; Compile`$155 = -b1x; Compile`$156 = b0x + Compile`$155; Compile`$157 = a0x*Compile`$156;
	Compile`$161 = -b1z; Compile`$163 = b0z + Compile`$161; Compile`$183 = -2*b0z*b1z;
	Compile`$184 = Compile`$20 + Compile`$62 + Compile`$140 + Compile`$98 + Compile`$183 + Compile`$177;
	Compile`$37 = Compile`$35*Compile`$36; Compile`$63 = Compile`$35*Compile`$62;
	Compile`$120 = -2*Compile`$35*b0y*b1y; Compile`$134 = Compile`$35*Compile`$133; Compile`$139 = a0z^2;
	Compile`$145 = Compile`$139*Compile`$142; Compile`$170 = -2*Compile`$35*b0z*b1z;
	Compile`$178 = Compile`$35*Compile`$177; Compile`$146 = a1z*Compile`$142; Compile`$182 = a0y^2;
	Compile`$185 = Compile`$182*Compile`$184; Compile`$186 = -b1y; Compile`$187 = b0y + Compile`$186;
	Compile`$196 = a1y*Compile`$184; 
	{
		(
			a0x*a1y*b0x*b0y + Compile`$37 - a0x*a1x*Compile`$36 + a0x*a1z*b0x*b0z +
			a1x*Compile`$36*b1x - a0x*a1z*b0z*b1x - a1z*b0x*b0z*b1x - a0x*Compile`$62*b1x + a1x*Compile`$62*b1x +
			a1y*b0y*Compile`$98 + a1z*b0z*Compile`$98 - a0x*a1y*b0x*b1y + a1y*Compile`$20*b1y + Compile`$120 +
			2*a0x*a1x*b0y*b1y + a0x*b0x*b0y*b1y - a1x*b0x*b0y*b1y - a1z*b0y*b0z*b1y + a1y*Compile`$62*b1y + a0x*a1y*b1x*b1y -
			a1y*b0x*b1x*b1y + a0x*b0y*b1x*b1y - a1x*b0y*b1x*b1y + Compile`$134 - a0x*a1x*Compile`$133 -
			a0x*b0x*Compile`$133 + a1x*b0x*Compile`$133 + a1z*b0z*Compile`$133 + Compile`$145 - a0x*a1z*b0x*b1z +
			a1z*Compile`$20*b1z + a1z*Compile`$36*b1z + Compile`$170 + 2*a0x*a1x*b0z*b1z + a0x*b0x*b0z*b1z -
			a1x*b0x*b0z*b1z - a1y*b0y*b0z*b1z + a0x*a1z*b1x*b1z - a1z*b0x*b1x*b1z + a0x*b0z*b1x*b1z - a1x*b0z*b1x*b1z -
			a1z*b0y*b1y*b1z - a1y*b0z*b1y*b1z + Compile`$178 - a0x*a1x*Compile`$177 - a0x*b0x*Compile`$177 +
			a1x*b0x*Compile`$177 + a1y*b0y*Compile`$177 - a0z*(-(a1x*b0x*b0z) - a1y*b0y*b0z + a1x*b0z*b1x - b0x*b0z*b1x +
			b0z*Compile`$98 + a1y*b0z*b1y - b0y*b0z*b1y + b0z*Compile`$133 + Compile`$146 +
			2*a0x*Compile`$156*Compile`$163 + a1x*b0x*b1z + Compile`$20*b1z + a1y*b0y*b1z + Compile`$36*b1z - a1x*b1x*b1z -
			b0x*b1x*b1z - a1y*b1y*b1z - b0y*b1y*b1z) + Compile`$185 -
			a0y*(-(a1x*b0x*b0y) + 2*a0z*b0y*b0z - a1z*b0y*b0z + a1x*b0y*b1x - b0x*b0y*b1x + b0y*Compile`$98 +
			2*a0x*Compile`$156*Compile`$187 + a1x*b0x*b1y + Compile`$20*b1y - 2*a0z*b0z*b1y + a1z*b0z*b1y +
			Compile`$62*b1y - a1x*b1x*b1y - b0x*b1x*b1y - 2*a0z*b0y*b1z + a1z*b0y*b1z - b0y*b0z*b1z + 2*a0z*b1y*b1z -
			a1z*b1y*b1z - b0z*b1y*b1z + b0y*Compile`$177 + Compile`$196)
		),
		(
			Compile`$13*Compile`$20 +
			Compile`$23*Compile`$20 + 2*a0x*a1y*b0x*b0y - 2*a1x*a1y*b0x*b0y + Compile`$37 - 2*a0x*a1x*Compile`$36 +
			Compile`$44*Compile`$36 + Compile`$23*Compile`$36 + 2*a0x*a1z*b0x*b0z - 2*a1x*a1z*b0x*b0z - 2*a1y*a1z*b0y*b0z +
			Compile`$63 - 2*a0x*a1x*Compile`$62 + Compile`$44*Compile`$62 + Compile`$13*Compile`$62 - 2*Compile`$13*b0x*b1x -
			2*Compile`$23*b0x*b1x - 2*a0x*a1y*b0y*b1x + 2*a1x*a1y*b0y*b1x - 2*a0x*a1z*b0z*b1x + 2*a1x*a1z*b0z*b1x +
			Compile`$13*Compile`$98 + Compile`$23*Compile`$98 - 2*a0x*a1y*b0x*b1y + 2*a1x*a1y*b0x*b1y + Compile`$120 +
			4*a0x*a1x*b0y*b1y - 2*Compile`$44*b0y*b1y - 2*Compile`$23*b0y*b1y + 2*a1y*a1z*b0z*b1y + 2*a0x*a1y*b1x*b1y -
			2*a1x*a1y*b1x*b1y + Compile`$134 - 2*a0x*a1x*Compile`$133 + Compile`$44*Compile`$133 + Compile`$23*Compile`$133 +
			Compile`$145 - 2*a0z*(Compile`$146 + (-(a1x*b0x) - a1y*b0y + Compile`$157 + a1x*b1x + a1y*b1y)*Compile`$163) -
			2*a0x*a1z*b0x*b1z + 2*a1x*a1z*b0x*b1z + 2*a1y*a1z*b0y*b1z + Compile`$170 + 4*a0x*a1x*b0z*b1z -
			2*Compile`$44*b0z*b1z - 2*Compile`$13*b0z*b1z + 2*a0x*a1z*b1x*b1z - 2*a1x*a1z*b1x*b1z - 2*a1y*a1z*b1y*b1z +
			Compile`$178 - 2*a0x*a1x*Compile`$177 + Compile`$44*Compile`$177 + Compile`$13*Compile`$177 + Compile`$185 -
			2*a0y*(Compile`$187*(Compile`$157 + a1x*(-b0x + b1x) + (a0z - a1z)*Compile`$163) + Compile`$196)
		)
	}
];

{s`numer, s`denom} = Simplify /@ Block[
	{
		Compile`$138, Compile`$165, Compile`$154, Compile`$164, Compile`$175,
		Compile`$168, Compile`$137, Compile`$194, Compile`$207, Compile`$213, Compile`$214, Compile`$215, Compile`$241,
		Compile`$220, Compile`$221, Compile`$222, Compile`$67, Compile`$69, Compile`$226, Compile`$227, Compile`$247,
		Compile`$248
	},
	Compile`$138 = b0x^2; Compile`$165 = b0y^2; Compile`$154 = a1z^2; Compile`$164 = a0x^2;
	Compile`$175 = b0z^2; Compile`$168 = a1x^2; Compile`$137 = a1y^2; Compile`$194 = b1x^2; Compile`$207 = b1y^2;
	Compile`$213 = -2*b0x*b1x; Compile`$214 = -2*b0y*b1y; Compile`$215 = Compile`$138 + Compile`$165 + Compile`$213 +
	Compile`$194 + Compile`$214 + Compile`$207; Compile`$241 = b1z^2; Compile`$220 = -b1x;
	Compile`$221 = b0x + Compile`$220; Compile`$222 = a0x*Compile`$221; Compile`$67 = -b0x;
	Compile`$69 = Compile`$67 + b1x; Compile`$226 = -b1z; Compile`$227 = b0z + Compile`$226; Compile`$247 = -2*b0z*b1z;
	Compile`$248 = Compile`$138 + Compile`$175 + Compile`$213 + Compile`$194 + Compile`$247 + Compile`$241;
	{
		(
			-(a0x*a1z*b0y) + a0x*a1y*b0z + a1z*b0y*b1x - a1y*b0z*b1x + a0x*a1z*b1y - a1z*b0x*b1y - a0x*b0z*b1y + a1x*b0z*b1y +
			a0z*(a1x*b0y - b0y*b1x + a1y*Compile`$69 - a1x*b1y + b0x*b1y) - a0x*a1y*b1z + a1y*b0x*b1z + a0x*b0y*b1z -
			a1x*b0y*b1z + a0y*(a1z*b0x - a1x*b0z - a1z*b1x + b0z*b1x + a1x*b1z - b0x*b1z)
		),
		(
			Compile`$137*Compile`$138 + Compile`$154*Compile`$138 + 2*a0x*a1y*b0x*b0y - 2*a1x*a1y*b0x*b0y +
			Compile`$164*Compile`$165 - 2*a0x*a1x*Compile`$165 + Compile`$168*Compile`$165 + Compile`$154*Compile`$165 +
			2*a0x*a1z*b0x*b0z - 2*a1x*a1z*b0x*b0z - 2*a1y*a1z*b0y*b0z + Compile`$164*Compile`$175 - 2*a0x*a1x*Compile`$175 +
			Compile`$168*Compile`$175 + Compile`$137*Compile`$175 - 2*Compile`$137*b0x*b1x - 2*Compile`$154*b0x*b1x -
			2*a0x*a1y*b0y*b1x + 2*a1x*a1y*b0y*b1x - 2*a0x*a1z*b0z*b1x + 2*a1x*a1z*b0z*b1x + Compile`$137*Compile`$194 +
			Compile`$154*Compile`$194 - 2*a0x*a1y*b0x*b1y + 2*a1x*a1y*b0x*b1y - 2*Compile`$164*b0y*b1y + 4*a0x*a1x*b0y*b1y -
			2*Compile`$168*b0y*b1y - 2*Compile`$154*b0y*b1y + 2*a1y*a1z*b0z*b1y + 2*a0x*a1y*b1x*b1y - 2*a1x*a1y*b1x*b1y +
			Compile`$164*Compile`$207 - 2*a0x*a1x*Compile`$207 + Compile`$168*Compile`$207 + Compile`$154*Compile`$207 +
			a0z^2*Compile`$215 - 2*a0z*(a1z*Compile`$215 + (-(a1x*b0x) - a1y*b0y + Compile`$222 + a1x*b1x + a1y*b1y)*
			Compile`$227) - 2*a0x*a1z*b0x*b1z + 2*a1x*a1z*b0x*b1z + 2*a1y*a1z*b0y*b1z - 2*Compile`$164*b0z*b1z +
			4*a0x*a1x*b0z*b1z - 2*Compile`$168*b0z*b1z - 2*Compile`$137*b0z*b1z + 2*a0x*a1z*b1x*b1z - 2*a1x*a1z*b1x*b1z -
			2*a1y*a1z*b1y*b1z + Compile`$164*Compile`$241 - 2*a0x*a1x*Compile`$241 + Compile`$168*Compile`$241 +
			Compile`$137*Compile`$241 + a0y^2*Compile`$248 -
			2*a0y*((b0y - b1y)*(Compile`$222 + a1x*Compile`$69 + (a0z - a1z)*Compile`$227) + a1y*Compile`$248)
		)
	}
];

par`numer = {
	ay*vx*vy - by*vx*vy - ax*vy^2 + bx*vy^2 + az*vx*vz - bz*vx*vz - ax*vz^2 + bx*vz^2,
	vy*(ax*vx - bx*vx + az*vz - bz*vz) - ay*(vx^2 + vz^2) + by*(vx^2 + vz^2),
	-(az*(vx^2 + vy^2)) + bz*(vx^2 + vy^2) + (ax*vx - bx*vx + ay*vy - by*vy)*vz
} /. Join[Thread[{ax, ay, az} -> tA0], Thread[{bx, by, bz} -> tB0], Thread[{vx, vy, vz} -> tB1]];

t1`numer`found = False;
t1`denom`found = False;
s`numer`found = False;
s`denom`found = False;
par`numer`found = False;
CheckScalar[expr_, array_] := Block[{high = Simplify[expr, TransformationFunctions :> {Automatic, TensorReduce}]},
	If[Not[aset["MemberQ", high]] && Not[NumberQ[high]],
		Block[{low = Simplify[high /. {A0->tA0, A1->tA1, B0->tB0, B1->tB1, Norm2[v_] :> Dot[v, v]}]},
			array["Append", high];
			If[low === t1`numer,
				Print["t1 numer found!"];
				Print[CForm[high]];
				ToString[CForm[high]] >>> "t1_numer.txt";
				Return[found`t1`numer]
			];
			If[low === t1`denom,
				Print["t1 denom found!"];
				Print[CForm[high]];
				ToString[CForm[high]] >>> "t1_denom.txt";
				Return[found`t1`denom]
			];
			If[low === s`numer,
				Print["s numer found!"];
				Print[CForm[high]];
				ToString[CForm[high]] >>> "s_numer.txt";
				Return[found`s`numer]
			];
			If[low === s`denom,
				Print["s denom found!"];
				Print[CForm[high]];
				ToString[CForm[high]] >>> "s_denom.txt";
				Return[found`s`denom]
			];
			If[low === par`numer,
				Print["par numer found!"];
				Print[CForm[high]];
				ToString[CForm[high]] >>> "s_denom.txt";
				Return[found`par`numer]
			];
			Nothing
		],
		Nothing
	]
];
CheckVector[expr_, array_] := Block[{high = Simplify[expr, TransformationFunctions :> {Automatic, TensorReduce}]},
	If[Not[aset["MemberQ", high]] && Not[NumberQ[high]],
		array["Append", high]
	];
	Nothing
];
DoUnary[{ops__}[arrins_] -> arrout_] := Block[{arrin = arrins[[-1]]}, If[Length[arrin] == 0, Nothing, {#}[arrin] -> arrout]& /@ {ops}];
DoBinary[{ops__}[arrins1_, arrins2_] -> arrout_] := Block[{size = Length[arrins1]},
	Flatten[Table[
		Block[{arrin1 = arrins1[[i]], arrin2 = arrins2[[size+1-i]]},
			If[Length[arrin1] > Length[arrin2],
				If[Length[arrin2] == 0,
					Nothing,
					Table[
						({#}[arrin1[[j]], arrin2] -> arrout)& /@ {ops},
						{j, Length[arrin1]}
					]
				],
				If[Length[arrin1] == 0,
					Nothing,
					Table[
						({#}[arrin1, arrin2[[j]]] -> arrout)& /@ {ops},
						{j, Length[arrin2]}
					]
				]
			]
		],
		{i, size}
	]]
];
Compute[{op_}[arrin_] -> Scalar] := Block[{tmp, found},
	tmp = CreateDataStructure["DynamicArray"];
	found = DeleteDuplicates[Table[
		CheckScalar[op[arrin[[i]]], tmp],
		{i, Length[arrin]}
	]];
	{tmp -> Scalar, found}
];
Compute[{op_}[in1_, arrin2_ /; DataStructureQ[arrin2]] -> Scalar] := Block[{tmp, found},
	tmp = CreateDataStructure["DynamicArray"];
	found = Table[
		CheckScalar[op[in1, arrin2[[i]]], tmp],
		{i, Length[arrin2]}
	];
	{tmp -> Scalar, found}
];
Compute[{op_}[arrin1_ /; DataStructureQ[arrin1], in2_] -> Scalar] := Block[{tmp, found},
	tmp = CreateDataStructure["DynamicArray"];
	found = Table[
		CheckScalar[op[arrin1[[i]], in2], tmp],
		{i, Length[arrin1]}
	];
	{tmp -> Scalar, found}
];
Compute[{op_}[arrin_] -> Vector] := Block[{tmp, found},
	tmp = CreateDataStructure["DynamicArray"];
	found = DeleteDuplicates[Table[
		CheckVector[op[arrin[[i]]], tmp],
		{i, Length[arrin]}
	]];
	{tmp -> Vector, found}
];
Compute[{op_}[in1_, arrin2_ /; DataStructureQ[arrin2]] -> Vector] := Block[{tmp, found},
	tmp = CreateDataStructure["DynamicArray"];
	found = Table[
		CheckVector[op[in1, arrin2[[i]]], tmp],
		{i, Length[arrin2]}
	];
	{tmp -> Vector, found}
];
Compute[{op_}[arrin1_ /; DataStructureQ[arrin1], in2_] -> Vector] := Block[{tmp, found},
	tmp = CreateDataStructure["DynamicArray"];
	found = Table[
		CheckVector[op[arrin1[[i]], in2], tmp],
		{i, Length[arrin1]}
	];
	{tmp -> Vector, found}
];
(* First level of scalars is empty *)
scalars = CreateDataStructure["DynamicArray", {CreateDataStructure["DynamicArray"]}];
points = {A0, A1, B0, B1};
vectors = {A1-A0, B1-B0, B0-A0} (* Flatten[(x |-> (If[x===#, Nothing, x-#]& /@ points)) /@ points] *);
Print["Vectors: ", Normal[vectors]];
vectors = CreateDataStructure["DynamicArray", {CreateDataStructure["DynamicArray", vectors]}];
aset = CreateDataStructure["HashSet", Flatten[(Normal /@ Normal[#])& /@ {scalars, vectors}]];
While[True,
	Block[{todo, results, newscalars, newvectors},
		newscalars = CreateDataStructure["DynamicArray"];
		newvectors = CreateDataStructure["DynamicArray"];
		Print[];
		Print["Layers: ", Length[vectors]];
		Print["Size: ", Length[aset]];
		Print["t1 numer found: ", t1`numer`found];
		Print["t1 denom found: ", t1`denom`found];
		Print["s numer found: ", s`numer`found];
		Print["s denom found: ", s`denom`found];
		Print["par numer found: ", par`numer`found];
		todo = Flatten[{
			DoUnary[{Minus}[scalars] -> Scalar],
			DoUnary[{Minus}[vectors] -> Vector],
			DoUnary[{Norm2}[vectors] -> Scalar],
			DoBinary[{Plus, Subtract, Times}[scalars, scalars] -> Scalar],
			DoBinary[{Cross, Plus, Subtract}[vectors, vectors] -> Vector],
			DoBinary[{Dot}[vectors, vectors] -> Scalar],
			DoBinary[{Times}[vectors, scalars] -> Vector]
		}];
		Print["In parallel section"];
		results = ParallelMap[
			Compute,
			todo
		];
		Print["Serializing"];
		Scan[
			Block[{es = #[[1]][[1]], kind = #[[1]][[2]], found = #[[2]]},
				Switch[kind,
					Scalar, (
						found /. {
							found`t1`numer :> (t1`numer`found = True),
							found`t1`denom :> (t1`denom`found = True),
							found`s`numer :> (s`numer`found = True),
							found`s`denom :> (s`denom`found = True),
							found`par`numer:> (par`numer`found = True)
						};
						Do[
							Block[{e = es[[i]]},
								If[Not[aset["MemberQ", e]],
									aset["Insert", e];
									newscalars["Append", e]
								]
							],
							{i, Length[es]}
						]
					),
					Vector, Do[
						Block[{e = es[[i]]},
							If[Not[aset["MemberQ", e]],
								aset["Insert", e];
								newvectors["Append", e]
							]
						],
						{i, Length[es]}
					]
				]
			]&,
			results
		];
		Print["nv: ", Length[newvectors], ", ns: ", Length[newscalars]];
		vectors["Append", newvectors];
		scalars["Append", newscalars]
	]
]




