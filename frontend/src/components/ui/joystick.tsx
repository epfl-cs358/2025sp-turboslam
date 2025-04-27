import React from "react";
import Joystick from "rc-joystick";

export function MyJoystick({
  onChange,
  className = "",
  ...props
}: {
  onChange?: (val: any) => void;
  className?: string;
  [key: string]: any;
}) {
  return (
    <div className={`p-2 ${className}`}>
      <Joystick
        baseRadius={60}
        controllerRadius={30}
        onChange={onChange}
        {...props}
      />
    </div>
  );
}
