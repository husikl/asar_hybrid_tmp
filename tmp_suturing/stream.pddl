; everything written here, should have a corresponding function!
(define (stream suture)

	(:stream s-motion
		:inputs ( ?arm ?pose2 )
		:domain (and  (IsArm ?arm) (IsPose ?pose2)  )
		:outputs (?control)
		:certified (and (Motion ?arm ?pose2 ?control)(IsControl ?control))
	)
	
	(:stream s-hold-motion
		:inputs ( ?arm ?grasp ?loc )
		:domain (and  (CanReachLocation ?arm ?grasp ?loc) )
		:outputs (?control)
		:certified (and (HoldMotion ?arm ?grasp ?loc ?control) (IsControl ?control))
	)

	(:stream s-grasp-traj
		:inputs ( ?arm ?pose )
		:domain (and  (IsArm ?arm) (IsPose ?pose) )
		:outputs (?control)
		:certified (and (GraspMotion ?arm ?pose ?control)(IsControl ?control))
	)

	(:stream s-grasp-pose
		:inputs ( ?arm ?location )
		:domain (and  (IsLocation ?location) (IsArm ?arm) )
		:outputs (?grasp)
		:certified (and (IKvalid ?arm ?location ?grasp ) (IsPose ?grasp))
	)

	(:stream s-reachability-loc
		:inputs ( ?arm ?loc ?grasp)
		:domain (and  (IsArm ?arm) (IsPose ?grasp) (IsLocation ?loc) )
		; :outputs (?grasp)
		:certified (and (CanReachLocation ?arm ?grasp ?loc))
	)

	(:stream s-insert-loc
		:inputs (?arm ?grasp ?loc)
		:domain (and  (IsLocation ?loc) (IsArm ?arm)  (IsPose ?grasp))
		; :outputs (?grasp)
		:certified (and (CanInsert ?arm ?grasp ?loc)  )
	)
	
	(:stream s-extract-loc
		:inputs (?arm ?grasp ?loc)
		:domain (and  (IsArm ?arm) (IsLocation ?loc) (IsPose ?grasp))
		; :outputs (?grasp)
		:certified (and (CanExtract ?arm ?grasp ?loc)   )
	)
	; (:stream s-extract-loc
	; 	:inputs (?arm  ?loc)
	; 	:domain (and  (IsArm ?arm) (IsLocation ?loc) )
	; 	:outputs (?grasp)
	; 	:certified (and (CanExtract ?arm ?grasp ?loc) (IsPose ?grasp)  )
	; )

	(:stream s-insert-motion
		:inputs (?arm ?grasp ?loc)
		:domain (and  (CanInsert ?arm ?grasp ?loc) )
		:outputs (?control)
		:certified (and (InsertMotion ?arm ?grasp ?loc ?control) (IsControl ?control))
	)

	(:stream s-extract-motion
		:inputs ( ?arm ?grasp ?loc )
		:domain (and  (IsArm ?arm) (IsPose ?grasp) (IsLocation ?loc) )
		:outputs (?control)
		:certified (and (ExtractMotion ?arm ?grasp ?loc ?control) (IsControl ?control))
	)

	
	(:function (Distance ?control)
    	(and (IsControl ?control))
  )
	
)
