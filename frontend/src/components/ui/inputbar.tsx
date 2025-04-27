import { InputHTMLAttributes } from "react";

type Props = InputHTMLAttributes<HTMLInputElement>;

export function MyInput({ className = "", ...props }: Props) {
  return (
    <input
      {...props}
      className={`anvil-textbox-active w-[var(--2-block)] p-1 text-black ${className}`}
    />
  );
}
